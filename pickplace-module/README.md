# pickplace-module

Viam module (Python) for armfarm19, the code-2 service. The wrist RealSense finds a 200 mm cube.
The xArm6 with its vacuum gripper carries it to a ring, into a box on the table, or sideways into a
shelf box on the wall, and brings it back home. Operating and deployment:
[DEPLOYMENT.md](DEPLOYMENT.md). How it all fits together geometrically: [GEOMETRY.md](GEOMETRY.md).

```
src/main.py                 module entry point (registers the model)
src/pickplace/
  service.py                code-2 generic service: DoCommand actions, background task, status
  cycle.py                  one cycle on real resources: look, plan, execute, verify; closed-loop suction, recovery
  planning.py               scene WorldState (boxes, ring, live walls/shelf, carried cube), concurrent plan requests, validation, crash retry
  task.py                   target poses for top and side entry; grip/yaw and wrist branch chosen with the placement
  memory.py                 where code-2 last put the cube (persisted under VIAM_MODULE_DATA)
  kinematics.py             FK/IK from the arm's own URDF; keeps joints away from their limits
  vision_geometry.py        aligned RGB-D cube/ring localisation and 5-frame consensus
  find_cube_ring.py         2D cube/ring detector
  calibration.py            fixed-overview calibration: camera XY -> tool XY, taught heights
  box.py                    box fit check and collision cavity (any number of named boxes)
                            side-entry shelf boxes are config (side_boxes) + frame-system panels
  box_survey.py             measure a new orange Post-it box from the camera (candidate config entry)
  grasp_geometry.py         cube <-> TCP transform shared by pickup and release
  transforms.py             Viam orientation-vector pose math
  config.py                 attribute validation (accepts the existing code-2 attributes)
config/code2-attributes.json   cleaned code-2 attributes (calibration, box, tuning)
tests/                      offline tests with saved overview frames and the live arm URDF
tools/                      status (read-only), plan_check (live plan-only), deploy, select_version
```

Pipeline per cycle: look (vision first, memory only as a fallback) → plan every stage → pick with
a closed-loop seal → carry → place → park at the overview → verify by camera. Alignment comes from
vision. Collision avoidance comes from the motion planner, which is given the cube, ring, table
patch, box walls and the live workcell obstacles.

```bash
.venv/bin/python -m pytest -q
```
