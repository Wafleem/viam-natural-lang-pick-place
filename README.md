# viam-natural-lang-pick-place

**NYU Ultraviolet** · a Viam robot that picks up a cube and puts it on a ring or into a box.

An xArm6 with a suction gripper and a wrist camera finds the white cube and plans a
collision-free path. It presses the suction cup into the cube, then places the cube on the black
ring or squarely inside an orange Post-it box.

| Folder | What's inside |
|---|---|
| [pickplace-module/](pickplace-module/) | The Viam module: vision, motion planning, the pick-and-place cycle. See [setup and usage](pickplace-module/DEPLOYMENT.md) and [the geometry](pickplace-module/GEOMETRY.md) |
| [dashboard/](dashboard/) | Ultraviolet test bench: buttons for every command, camera overlay, arm/suction debug, tuning, boxes, plan checks, versions, logs |
| [nl-integration/](nl-integration/) | Plain language → robot commands, plus the JSON API schema for web apps |

```bash
make setup       # Python venv with everything
make test        # all offline tests
make dashboard   # http://localhost:8765
make help        # everything else
```

Or straight from the Viam app (CONTROL → code-2 → DoCommand):

```json
{"action": "pick_place", "pick": {"object": "cube", "select": "left"}, "place": {"color": "orange"}, "workcell_ready": true}
```
