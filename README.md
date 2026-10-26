# viam-natural-lang-pick-place

**NYU Ultraviolet** · built at the **Viam office in New York City** during the **Code That Moves
Hackathon** by **Safal, Sunny and Josiah**.

Tell a robot arm where a cube should go, in plain words, and it goes there:

> "put the cube in the box on the wall" · "put it on the ring" · "reset the cube"

An xArm6 with a vacuum gripper and a wrist-mounted RealSense D435 finds a 200 mm cube with
computer vision. It plans a collision-free path through a modelled workcell and presses the
suction cups into the cube. Then it places the cube on a ring, into a box on the table, or slides
it sideways into a shelf box on the wall. A local dashboard drives all of it with buttons, typed
text or your voice.

![The Ultraviolet dashboard](docs/dashboard-run.jpg)

## 1. Computer vision with the cube

The camera rides on the wrist, so the arm moves to look at things before it moves to touch them.

- **Finding the cube.** The detector first groups the printed corner marks on the cube's faces in
  the colour image. That gives a rough cube region, which is refined in 3D from the aligned depth
  image:
  - fit the horizontal plane at cube-top height,
  - grow the connected coplanar face,
  - fit a rectangle to it, giving centre, yaw and measured size.

  Five consecutive frames must agree before anything moves
  ([`vision_geometry.py`](pickplace-module/src/pickplace/vision_geometry.py),
  [`find_cube_ring.py`](pickplace-module/src/pickplace/find_cube_ring.py)).
- **From pixels to the arm.** A calibration taught at one overview pose maps camera-world points
  to tool coordinates. Heights come from taught contacts, not raw depth
  ([`calibration.py`](pickplace-module/src/pickplace/calibration.py)).
- **Seeing past the edge of the frame.** If the cube isn't fully in view, the arm *sweeps*: it
  translates the camera sideways at the same height and orientation. Because a sweep view is a
  pure translation of the overview camera, a point seen there maps through the same calibration:
  `tool = cal(raw − shift) + shift`. An earlier version anchored on the ring, at the wrong height,
  and was about 25 mm off. This mapping predicted the cube's real position to within 1 mm.
- **Deterministic grip and orientation.** The cup's pose on the cube is stored as a transform,
  `G = C_pick⁻¹ · T_pick`. Every release is `T_release = C_goal · G`, so the cube lands in a
  known orientation. The cube's 90° symmetry is used to pick the grip with the least joint travel,
  with pick and place on the same wrist branch, so the wrist never flips while carrying
  ([`task.py`](pickplace-module/src/pickplace/task.py),
  [`grasp_geometry.py`](pickplace-module/src/pickplace/grasp_geometry.py)).
- **Vision first, every time.** Every command looks before it moves, because people move the cube
  by hand. Memory of where code-2 last put the cube is only a fallback, for when the cube sits in
  a receptacle the overview can't judge.
- **A camera swap mid-event.** When the RealSense was replaced, we fitted a depth-scale and mount
  correction from fixed features on the table: tape marks, blue tape and the ring. The old
  calibration then stayed valid. Measuring the same box from two camera distances settled which
  camera's depth was right.

## 2. Receptacles in the scene

Each place the cube can go is a modelled object that the motion planner respects. Carrying the
cube makes it part of the arm.

| Place | How it's modelled | How the cube goes in |
|---|---|---|
| **Ring** | A fixed spot, as four bars covering the disc | Top-down, seated on the rim |
| **Box on the table** (orange) | Measured by the camera from its wall planes: floor, inner size, yaw, rim height; a floor slab plus four walls | Top-down through a 203 mm opening, about 1.6 mm per side |
| **Box on the wall** (pink) | An open shelf of five panels (floor, top, back, left, right) in the machine's frame system, checked against the camera | **Sideways**: turned level, slid straight in, released 8 mm from the back, then the arm backs straight out |
| **Home spot** | A taught table position | Top-down |

- **Workcell obstacles are live.** The walls, the ceiling and the shelf panels are read from the
  Viam frame system every cycle, so a teammate can recalibrate them in the app and the next move
  uses the new positions.
- **The carried cube is collision-checked.** It's attached to the gripper in the plan request, and
  every carrying path is swept through the cube's real volume to check its clearance. Carries to
  table goals keep the cube upright, so a 200 mm cube can't swing out sideways.
- **Placements were taught with *hover trials*.** The arm holds the cube just outside a receptacle;
  a person says "half an inch to the right", the config is adjusted, and only then does it push the
  cube in ([`side_boxes`, `boxes`, `workcell_obstacles` in the config](pickplace-module/config/code2-attributes.json)).
- **Reset works from anywhere.** "Reset the cube" brings it home from the ring, either box, or
  anywhere on the table, including sliding it back out of the wall shelf.
- **Failures are recoverable.** The vacuum gripper's holding sensor confirms the seal and checks
  the grip after the lift, with up to three re-picks. If a run fails while the cube is still held,
  code-2 sets the cube back down where it picked it up, instead of freezing mid-air.

## 3. The local dashboard and natural language

[`dashboard/`](dashboard/) is a local web page that served as our debug bench and as the
natural-language front end.

- **Debug bench:**
  - big flow buttons for every route, plus a full tour, hover tests and recovery;
  - live wrist camera;
  - joint table with limit bars and jogs;
  - suction and controller diagnostics;
  - a config editor;
  - live plan-only checks;
  - module versions with one-click deploy;
  - robot logs.

  Anything that moves needs **workcell clear** ticked, and **STOP** is always one click away.
- **Natural language:** hold-to-talk voice (browser speech recognition) or typed text →
  [`nl-integration/nl_to_command.py`](nl-integration/nl_to_command.py) → a reviewed code-2
  command.
  - The parser is rule-based and tolerant of mishearings ("rink" → ring, "books" → box).
  - It speaks back what it understood, with a choice of voices.
  - Nothing moves until you press **Send to robot**.
  - It follows the command to the end and tells you if it was interrupted.

| Say | What happens |
|---|---|
| "put the cube in the orange box" / "the box on the table" | cube → table box |
| "put the cube in the pink box" / "the box on the wall" / "on the shelf" | cube → wall shelf box (sideways entry) |
| "put the cube on the ring" | cube → ring |
| "reset the cube" / "take the cube home" | fetch it from wherever it is → home spot |
| "the cube is home" / "I put the cube back" | only *records* that it's home (nothing moves) |
| "go home" · "status" · "let go" · "stop" | arm to the overview · report · suction off · stop now |

For web apps and language models, [`nl-integration/command_schema.json`](nl-integration/command_schema.json)
describes every command, and `{"action": "describe"}` returns the live goals.

## Repository

| Folder | What's inside |
|---|---|
| [pickplace-module/](pickplace-module/) | The Viam module, code-2: vision, planning, the pick-and-place cycle. See [operating guide](pickplace-module/DEPLOYMENT.md) and [geometry](pickplace-module/GEOMETRY.md) |
| [dashboard/](dashboard/) | The local test bench and voice front end |
| [nl-integration/](nl-integration/) | Plain language → commands, plus the JSON command schema |

```bash
make setup       # Python venv with everything
make test        # every offline test suite (no robot needed)
make dashboard   # http://localhost:8765 (needs `viam login` and the machine online)
make help        # everything else
```

Or straight from the Viam app (CONTROL → code-2 → DoCommand):

```json
{"action": "pick_place", "pick": {"object": "cube", "select": "only"}, "place": {"goal": "pink"}, "workcell_ready": true}
```
