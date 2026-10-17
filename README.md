# viam-natural-lang-pick-place

A Viam robot that picks up a cube and puts it on a ring or into a box.

An xArm6 with a suction gripper and a wrist camera finds the white cube, plans a collision-free
path, presses the suction cup into the cube, and places the cube on the black ring or inside an
orange Post-it box.

| Folder | What's inside |
|---|---|
| [pickplace-module/](pickplace-module/) | The Viam module: vision, motion planning and the pick-and-place cycle ([setup and usage](pickplace-module/DEPLOYMENT.md)) |
| [nl-integration/](nl-integration/) | Coming soon: control the robot in plain language |

Example, from the Viam app (CONTROL → code-2 → DoCommand):

```json
{"action": "box_once", "workcell_ready": true}
```
