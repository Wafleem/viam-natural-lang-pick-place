# Operating and deployment guide: armfarm19 pick-and-place (code-2)

code-2 is a Viam generic service on **armfarm19** (org *hackathons*, location *Arm Farm*):

- module `hackathons:armfarm19-vision-pickplace`
- model `hackathons:armfarm19-vision-pickplace:generic-service`
- machine `armfarm19`, part `armfarm19-main` (`f5c74a65-a023-4983-a317-1e9a88c88372`)
- hardware: UFACTORY xArm6 (`viam:ufactory`), vacuum gripper `Vgripper`, wrist RealSense D435 `cam`

The last version demonstrated on hardware is **0.0.58**. The geometry behind every step is in
[GEOMETRY.md](GEOMETRY.md).

## 1. Setup on your computer

From the repo root: `make setup`, `make test`, `make dashboard` (`make help` lists everything).

```bash
cd pickplace-module
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt -r requirements-dev.txt
viam login        # browser OAuth; the tools read your CLI token and never store a key
```

If `viam` is not on your PATH, set `VIAM_CLI=/path/to/viam`.

## 2. Check without moving anything

| Command | What it checks |
|---|---|
| `.venv/bin/python -m pytest -q` | Kinematics, vision on saved frames, targets, planning, a full fake-robot cycle, retries and recovery, the API |
| `.venv/bin/python tools/plan_check.py --goal box` | Every stage on the **live** motion planner (plan requests only, nothing moves) |
| `.venv/bin/python tools/status.py` | Joints, TCP, joint-limit clearance, code-2 status |

## 3. Deploy and roll back

```bash
.venv/bin/python tools/deploy.py --version 0.0.59             # upload + cloud build (linux/amd64)
.venv/bin/python tools/select_version.py --version 0.0.59          # preview
.venv/bin/python tools/select_version.py --version 0.0.59 --apply  # switch code-2
.venv/bin/python tools/select_version.py --version 0.0.58 --apply  # roll back
```

- `select_version` changes only the code-2 module version and service attributes. `--set
  KEY=JSON` merges single attributes, and `--attributes file.json` replaces them all.
- It refuses while the arm moves or code-1/code-2 are busy, and it verifies the readback.
- The previous entry is saved locally to `tools/config-before-<version>.json` (gitignored).
- **Selecting a version restarts code-2 for a few seconds.** A command sent during that window is
  lost. The dashboard reports it as interrupted; just send it again.

## 4. Operating

Use the dashboard ([../dashboard](../dashboard/)), or the Viam app → armfarm19 → CONTROL → **code-2** →
DoCommand. Commands that move need `"workcell_ready": true`. Long commands return at once; poll
`{"action":"status"}`.

| Command | Moves? | What it does |
|---|---|---|
| `{"action":"pick_place","pick":{"object":"cube","select":"only"},"place":{"goal":"pink"}}` | **yes** | One cycle to a goal: `home`, `ring`, `box`, `pink`, or `{"type":"ring"}` / `{"color":"orange"}` |
| `{"action":"reset_cube"}` | **yes** | Brings the cube home from wherever it is: a box, the shelf, the ring or the table |
| `{"action":"hover_trial","goal":"pink","hover_s":30}` | **yes** | Holds the cube just outside a goal for a person to judge, then sets it back where it was picked |
| `{"action":"recover_cube"}` | **yes** | After a failure: sets a still-gripped cube back on its pick spot and parks |
| `{"action":"scan"}` | camera | Overview plus sweep; reports where the cube is |
| `{"action":"reset"}` | **yes** | Suction off and park at the overview |
| `{"action":"recover_limits"}` | **yes** | Rolls back a joint sitting at its hard limit |
| `{"action":"set_cube_location","place":"home"}` | no | After moving the cube by hand: `unknown`, `table`, `home` or a goal |
| `{"action":"dry_run","goal":"box"}` | no | Detect and plan every stage; reports clearances |
| `{"action":"measure_box","name":"box2"}` | no | Measures a new orange Post-it box → candidate `boxes` entry |
| `{"action":"status"}` · `describe` · `self_test` · `release` · `stop` | no | State and events · API and goals · checks without motion · suction off · cancel and stop the arm |

### One cycle

1. **Look.** Go to the overview and find the cube by vision, sweeping if needed. When code-2
   remembers the cube in a receptacle, it takes one overview look only, then uses the remembered
   placement.
2. **Plan.** Plan every stage. Independent chains are planned concurrently, and internal
   motion-service crashes are retried.
3. **Pick.** Approach, descend, suction on, press `press_depth_mm` into the cube at 1°/s.
4. **Seal.** Wait for the holding sensor to report a seal, up to `suction_dwell_s`, then
   `seal_settle_s`.
5. **Lift.** Check the holding sensor again. If there's no seal or a drop, re-pick, up to
   `pick_retries`.
6. **Carry.** Carry and place. Table goals keep the cube upright; the shelf turns it level and
   slides it in.
7. **Release.** Suction off, retract, park at the overview.
8. **Check.** Verify by camera that the pick spot is empty.

A failure with the cube still held triggers automatic recovery: the cube is set back on its pick
spot and the arm parks. **STOP** stops in place. Keep the physical e-stop in reach; the software
stop is not an e-stop.

### Places (config)

All coordinates are in the arm's world frame, in mm; +x points from the arm toward the wall.

| Key | What it describes |
|---|---|
| `home` | The cube's table spot `{"xy_mm", "yaw_deg"}` |
| `ring` | Fixed ring spot `{"xy_mm", "diameter_mm", "height_mm"}` |
| `boxes` | Top-entry boxes: floor centre, inner size, yaw, rim, optional `placement_center_xy_mm`, `color`, `label` (the orange box is `"box on the table"`) |
| `side_boxes` | Side-entry boxes (the pink wall shelf): `back_x_mm`, `floor_z_mm`, `center_y_mm`, `opening_y_mm`, `top_z_mm`, `front_x_mm`, `panels` (frame names), `back_clearance_mm`, `release_above_floor_mm`, `entry_margin_mm`, `roll_deg` |
| `workcell_obstacles` | Frame-system obstacles the carried cube must clear: `[{"frame", "dims_mm"}]`. Poses are read live every cycle |
| `camera_correction` | Replacement camera → calibrated camera: `depth_scale` and a 4×4 `matrix` |

**Teaching a placement.** Run `hover_trial` and have someone say which way and how far the cube is
off. Adjust the entry (for example the box's floor centre, or the shelf's `center_y_mm`, `roll_deg`
or `release_above_floor_mm`) with `select_version --set`, and hover again before the first real
placement.

**Adding a top-entry box.** Make the box visible from the overview, with two orange Post-its, then
`reset` and `measure_box`. Check the candidate, add it to `boxes`, then `dry_run` → `hover_trial` →
`pick_place`.

**Adding a side-entry box.** Model its panels in the machine config, and add each to
`workcell_obstacles`. Point the camera into the opening and measure the floor, back and sides from
depth (as in GEOMETRY.md), then add a `side_boxes` entry and hover-test.

### Tuning

| Key | Demo value | Range |
|---|---|---|
| `press_depth_mm` / `press_speed_deg_s` | 20 / 1 | 0–20 / 0.2–5 |
| `suction_dwell_s` (longest wait for the seal) / `seal_settle_s` | 3.0 / 0.5 | 0–10 / 0–5 |
| `pick_retries` | 3 | 0–5 |
| `travel_speed_deg_s` / `approach_speed_deg_s` / `sweep_speed_deg_s` | 75 / 37.5 / 75 | ≤100 / ≤50 / ≤100 |
| `acceleration_deg_s2` | 94 | 1–200 |
| `clearance_mm` (hover above a table, ring or home contact) | 50 | 30–300 |
| `carry_clearance_mm` / `carry_upright_deg` | 50 / 15 | the planner keeps this from walls and ceiling / max tilt while carrying to table goals |
| `joint_limit_margin_deg` | 5 | 1–30 |

The arm driver's own cap is `speed_degs_per_sec` in the machine config (75 at the demo). The xArm's
collision sensitivity was 0 for the 20 mm press; the planner and the closed-loop suction carry the
safety load.

## 5. What has and has not been verified

**On hardware, Oct 17, 2026, by the end of the event, with 0.0.54–0.0.58:**
- cube → table box, box → home, cube → ring, ring → home, cube → wall shelf (sideways),
  shelf → home, and full tours;
- each driven by buttons and by natural-language commands from the dashboard;
- closed-loop seal and lift checks;
- a mid-air failure recovered by setting the cube back down;
- the replacement camera after its correction was fitted.

**Offline only (after the hardware was no longer available):**
- the planner-crash retry moved into `Planner.segment`, so it covers every plan call, with a test
  that injects the crash;
- the JSON command schema, with a test that keeps it in sync with the service;
- wording and documentation updates.

**Known limits:**
- Inside receptacles, the cube's pose comes from code-2's memory of where it put the cube, not from
  a fresh camera measurement. If someone takes it out by hand, run `set_cube_location`.
- Sweep views at the far right sometimes read the cube 12–13 mm high and fall through to the next
  view. That costs a few seconds, not correctness.
- Carries at 2.5× speed rely on the seal; the holding sensor is checked after the lift, not during
  the carry.
