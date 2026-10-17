# pickplace-module

Viam module (Python) for armfarm19: the wrist RealSense finds the white RoboMaster cube, and the
xArm6 with its vacuum gripper carries it to the black ring or into the orange Post-it box.
Deployment and operation: [DEPLOYMENT.md](DEPLOYMENT.md).

```
src/main.py                 module entry point (registers the model)
src/pickplace/
  service.py                code-2 generic service: DoCommand actions, background task, status
  cycle.py                  one cycle on real resources: reset, detect, plan, execute; open-loop suction
  planning.py               scene WorldState, plan requests to the builtin motion service, plan validation
  task.py                   target poses (pickup, 10 mm press, release) and yaw choice by joint travel
  kinematics.py             FK/IK from the arm's own URDF; keeps joints away from their limits
  vision_geometry.py        aligned RGB-D cube/ring localisation and 5-frame consensus
  find_cube_ring.py         2D cube/ring detector
  calibration.py            fixed-overview calibration: camera XY -> tool XY, taught heights
  box.py                    box fit check and collision cavity (any number of named boxes)
  box_survey.py             measure a new orange Post-it box from the camera (candidate config entry)
  grasp_geometry.py         cube <-> TCP transform shared by pickup and release
  transforms.py             Viam orientation-vector pose math
  config.py                 attribute validation (accepts the existing code-2 attributes)
config/code2-attributes.json   cleaned code-2 attributes (calibration, box, tuning)
tests/                      offline tests with saved overview frames and the live arm URDF
tools/                      status (read-only), plan_check (live plan-only), deploy, select_version
```

Pipeline per cycle: `Grab` and `Open` are the only gripper calls. Alignment comes from vision,
and collision avoidance from the motion planner, which is given the cube, ring, local table patch
and box walls.

```bash
.venv/bin/python -m pytest -q
```
