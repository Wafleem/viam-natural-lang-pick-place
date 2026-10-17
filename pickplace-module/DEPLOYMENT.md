# Deployment guide — armfarm19 pick-and-place (code-2)

This module replaces code-2 version 0.0.24 on **armfarm19** (org *hackathons*, location *Arm Farm*).
It keeps the same registry module and model, so the existing `code-2` service config keeps working:

- module `hackathons:armfarm19-vision-pickplace`
- model `hackathons:armfarm19-vision-pickplace:generic-service`
- machine `armfarm19`, part `armfarm19-main` (`f5c74a65-a023-4983-a317-1e9a88c88372`)

## What changed from 0.0.24

| Area | 0.0.24 | This module |
|---|---|---|
| Suction | Required 3 consecutive `IsHoldingSomething = true` readings; polled it during carry and aborted on false/RPC error | **Open loop.** `Grab` and dwell `suction_dwell_s`, `Open` and dwell `release_dwell_s`. The holding sensor is never read (a test enforces this) |
| Pickup depth | Taught contact + up to three 1 mm retries | Descend to 20 mm above contact, suction on, **press 10 mm** (`press_depth_mm`) into the cube at `press_speed_deg_s` |
| Wrist / J4 | Planner picked J4 = 360.000° (the URDF bound); the controller faulted with *Joint Angle Exceeds Limit* | IK picks free-space joint goals near the current pose; **every planned node keeps 5° from every joint limit** (`joint_limit_margin_deg`). J4 now stays near 0° |
| Cube yaw | Fixed pickup branch | The square cube allows 90° steps; the step with the least joint travel is chosen for pickup and placement |
| Targets | Ring + box survey flows, legacy sweep commands | One flow for both goals: vision at the calibrated overview → plan every stage → execute |
| Ring | Required for every goal | Required for `ring`; optional obstacle for box goals |
| Boxes | One hard-wired box | `boxes` list of named goals; `measure_box` drafts new entries from the camera |
| Diagnostics | — | `self_test` checks kinematics, camera, lease and the reset path without moving |

Unchanged: vision targeting (5-frame RGB-D consensus, overview calibration, 15 mm grasp offset toward
the image top), the accepted box pose/aperture/floor, collision-checked planning in the Viam builtin
motion service, the code-1 arm lease, slow vertical moves.

## 0. Physical state to fix first

The last 0.0.24 run left the arm stopped at the failed lift. The latest read showed:

- e-stop pressed in (`xArm: Emergency Stop Button Pushed In`)
- J4 = **360.0008°**, just past its limit; TCP ≈ (377, 240, 195) mm, above the cube

Steps, at the robot:

1. Make sure nobody is in the workcell. Release the e-stop (it latches; this can't be done remotely).
2. From here everything can be done remotely. In the dashboard's *xArm controller* panel, press
   `clear_error`. That is the `viam:ufactory` driver's `{"clear_error": true}` arm DoCommand. xArm
   Studio's *Clear Error* → *Enable* works too.
3. Send `{"action":"recover_limits","workcell_ready":true}`. It clears the error again, switches
   suction off and rolls **only J4** back to 355° at 2°/s. The tool can shift by ~15 mm. If any joint
   is more than 6° past its limit, the command refuses; jog the arm by hand.
4. Send `{"action":"reset","workcell_ready":true}`. The arm retreats straight up, then goes to the
   calibrated overview.

## 1. One-time setup on your computer

```bash
cd ~/Documents/armfarm-pickplace/pickplace-module
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt -r requirements-dev.txt
viam login        # browser OAuth; the tools read your CLI token, never a stored key
```

If `viam` is not on your PATH, set `VIAM_CLI=/path/to/viam`. `VIAM_CLI_CONFIG` overrides the CLI
config file (default `~/.viam/cached_cli_config.json`).

## 2. Check without moving anything

```bash
.venv/bin/python -m pytest -q
```

Offline: kinematics against the live frame system, IK limit margins, perception on saved
overview frames, the 10 mm press target, box fit, a full fake-robot cycle (order, speeds, no
holding-sensor reads), the guards on the service commands.

```bash
.venv/bin/python tools/plan_check.py --goal box
```

```bash
.venv/bin/python tools/plan_check.py --goal ring --ring-world-xy 440 130
```

These plan every stage on the **live** motion planner (real frame system and obstacles). The
start state is the overview and the observation is a saved frame. Only `plan` requests are sent.
`--ring-world-xy` moves the saved ring to a what-if position. `--live-detect` uses the camera
instead; the arm must be parked at the overview.

```bash
.venv/bin/python tools/status.py
```

Read-only: joints, TCP, joint-limit clearance, code-2 status.

## 3. Deploy

```bash
.venv/bin/python tools/deploy.py --version 0.0.25
```

This uploads the source and starts the cloud build (linux/amd64). Wait for **Done** at
<https://app.viam.com/module/hackathons/armfarm19-vision-pickplace> (Builds).

```bash
.venv/bin/python tools/select_version.py --version 0.0.25
```

Preview: shows the current and new version and the changed sections.

```bash
.venv/bin/python tools/select_version.py --version 0.0.25 --apply
```

This changes only the code-2 module version. It refuses while the arm moves or code-1/code-2 are
busy, and verifies the readback. The existing code-2 attributes are compatible: new keys take
their defaults (10 mm press, 1 s dwells, 10/3/2 °/s, 5° margin).

Optional: replace the attributes with the cleaned set in `config/code2-attributes.json` by adding
`--attributes config/code2-attributes.json`. The previous module/service entry is saved to
`tools/config-before-<version>.json`. If you replace attributes, roll back with that file, because
0.0.24 needs its old attributes.

Then confirm in the app (CONTROL → code-2 → DoCommand) that status reports
`"implementation": "pickplace-v1-open-loop-suction-press"`.

### Roll back

```bash
.venv/bin/python tools/select_version.py --version 0.0.24 --apply
```

Version-only rollback works when the attributes were not replaced.

## 4. Operating

Viam app → armfarm19 → CONTROL → **code-2** → DoCommand. Commands that move the arm need
`"workcell_ready": true`. Long commands return immediately; poll `status`.

| Command | Moves? | What it does |
|---|---|---|
| `{"action":"status"}` | no | state, stage, error, goals, plan summary, last 30 events |
| `{"action":"self_test"}` | no | from any pose: kinematics vs frame system, camera, lease, reset plan. No gripper calls |
| `{"action":"dry_run","goal":"box"}` | no | at the overview: detect + plan every stage, report clearances |
| `{"action":"measure_box","name":"box2"}` | no | at the overview: measure an orange Post-it box → candidate `boxes` entry |
| `{"action":"box_once","workcell_ready":true}` | **yes** | one cube → first configured box (add `"goal":"box2"` for another) |
| `{"action":"ring_once","workcell_ready":true}` | **yes** | one cube → ring cycle |
| `{"action":"run","goal":"<ring or box name>","workcell_ready":true}` | **yes** | one cycle to any goal |
| `{"action":"reset","workcell_ready":true}` | **yes** | suction off, straight retreat if low, go to the overview |
| `{"action":"recover_limits","workcell_ready":true}` | **yes** | roll back a joint sitting at its hard limit |
| `{"action":"release"}` | no | suction off |
| `{"action":"stop"}` | stops | cancels the command and stops the arm (not an e-stop) |

A cycle: suction off → overview → detect → plan all stages → approach (10°/s) → descend to 20 mm
above the cube (3°/s) → **suction on** → press to 10 mm below the taught contact (2°/s) → dwell 1 s
→ lift (3°/s) → transfer (10°/s) → place (3°/s) → **suction off**, dwell 1 s → retract → overview.

On any failure the arm stops and **suction is left as it was**, so a carried cube is not dropped.
Check the workcell, then use `release` and/or `reset`.

Keep the physical e-stop in reach. The software `stop` is not an e-stop.

### Workcell rules

- **Boxes**: each must stay where it was accepted. The current `box` has floor centre
  (298.2, −143.1, 0.85) mm, yaw −4.93°, opening 261.4 × 206.6 mm and rim z 130.0 mm. If someone moves a
  box, re-measure it and update its entry. Every configured box is a collision obstacle for every goal.
- **Ring**: the cube resting on the ring must not reach the box or `wall-front` (x ≈ 600 mm).
  Verified on the live planner: ring at tool XY (440, 130) plans; (530, 67), where it sat at 23:02 on
  Oct 16, does not (cube would hit `wall-front`), and (440, 60–90) does not (box wall).
- **Cube and ring** must be fully visible from the overview. The ring is optional for the box goal.

### Tuning (code-2 attributes)

| Key | Default | Range |
|---|---|---|
| `press_depth_mm` | 10 | 0–20 |
| `pre_contact_mm` | 20 | 5–100 |
| `suction_dwell_s` / `release_dwell_s` | 1.0 | 0–10 |
| `travel_speed_deg_s` / `approach_speed_deg_s` / `press_speed_deg_s` | 10 / 3 / 2 | 0.5–20 / 0.5–10 / 0.2–5 |
| `joint_limit_margin_deg` | 5 | 1–30 |
| `clearance_mm` | 100 | 30–300: approach height above the cube / ring contact |
| `box_entry_clearance_mm` | 30 | 10–200: carried cube bottom above a box rim before entering |
| `release_above_goal_mm` | 0 | 0–50: release this far above the seated height |
| `default_goal` | from config | `ring` or a box name |
| `lease_service` | `code-1` | `""` disables the lease |

### Adding another box

Goals are `ring` plus one goal per entry in the `boxes` list. The older single `box` /
`surveyed_box.box` attributes are still read as one box named `box`.

```json
"boxes": [
  {"name": "box",  "floor_center_world_mm": [298.2, -143.1, 0.85], "placement_center_xy_mm": [298.4, -141.3],
   "inner_dimensions_mm": [261.4, 206.6], "world_yaw_deg": -4.93, "rim_z_mm": 130.0},
  {"name": "box2", "...": "from measure_box"}
]
```

1. Put the new box where the overview camera sees all four walls and its inside floor, with two
   orange Post-its on it. Run `reset`, then `{"action":"measure_box","name":"box2"}`.
2. `status.last_result.candidate` is a ready-made entry: three frames that agree, the tightest
   opening seen, and heights matched to the taught tabletop.
3. Check the opening with a tape measure. Add the entry to `boxes`, then run
   `{"action":"dry_run","goal":"box2"}` before the first `run`.

The cube needs a few mm of clearance per side (it is placed with no padding). The carry height
clears the tallest configured rim.

After `return`, the cycle checks the pick spot with the camera, since there's no usable suction
feedback. `last_result.outcome.cube_left_pick_spot` is `false` when the cube is still there.

The xArm's collision sensitivity is 3. If the controller reports a collision during `press`, lower the
sensitivity in xArm Studio, or reduce `press_depth_mm`/`press_speed_deg_s`.

## 5. What has and has not been verified

Verified without motion (night of Oct 16–17, 2026):
- 48 offline tests pass (pickplace-module), plus 11 parser tests (nl-integration).
- FK from the live URDF matches the live frame system to 1e-6 mm.
- Live motion planner: every stage of a cube → box cycle plans from the overview. The wrist stays
  on the taught branch (J4 180–191.5°) like the operator-taught cycle; 0.0.24 drove J4 to 360°.
- Live motion planner: cube → ring plans with the ring at (440, 130).
- Deployed on armfarm19, now **0.0.30** (cloud build Done): `status`, `describe` and `self_test`
  run on the robot. self_test passes kinematics, camera (intrinsics match calibration) and lease.
  The reset plan is correctly blocked until `recover_limits`, because J4 sits at 360.0008°.

- Teammate web app (branch `integrate-web-ui`, `code3-web/`): moved to `pick_place`. 51/51 tests on
  Node 22. Fed the robot's live status it unlocks Send for ring, box and orange-box tasks and blocks
  a blue box. The robot parses its exact `pick`/`place` arguments (checked with `dry_run`).

Not verified:
- Any physical execution of this module (the e-stop has been latched the whole time).
- Whether a 10 mm press with open-loop suction holds the cube through transfer.
- Whether the controller tolerates the press at collision sensitivity 3.
- The camera/overview calibration and the accepted box pose. These are unchanged from 0.0.24 and
  only as good as before.

Worth checking with someone on site:
- **Vacuum feedback wiring.** `Vgripper` has no `connection_type`, so the driver assumes
  `contact` wiring on this arm (hardware ≥1305). If the cup is the older plug-in type, the holding
  signal is read from the wrong inputs. With a cube sealed, compare the dashboard's
  *contact* / *plugin* vacuum readings. code-2 does not depend on this either way.
- **Front wall obstacle.** A fragment override moves `wall-front` from 740 to 650 mm. It's what
  rules out ring positions near x ≈ 530. If the real wall is farther out, that override is too
  conservative.
