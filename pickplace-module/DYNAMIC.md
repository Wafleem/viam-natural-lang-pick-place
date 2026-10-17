# dynamic-scene branch

On `main`, the cube and ring are already found again at the overview on every cycle. Boxes are
fixed config entries. This branch lets boxes move too.

| Setting | Effect |
|---|---|
| `"box_tracking": "cached"` (default) | boxes keep their configured pose (same as `main`) |
| `"box_tracking": "live"` | every box is re-measured at the overview each cycle |
| `"track": true` on a box entry | only that box is re-measured |

How tracking works (`src/pickplace/scene.py`):
- It reuses the five overview frames the cube detection already captured, so there are no extra moves.
- Each frame is run through the orange Post-it box survey (`box_survey.measure_box`).
- A measurement is matched to the configured box within 300 mm. At least 3 frames must agree
  within 4 mm and 1.5°.
- Only the pose changes: floor-centre XY, yaw and the placement point move rigidly. The accepted
  opening, floor and rim heights stay as configured, so one noisy view can't shrink the cavity.
- A box the camera can't see keeps its configured pose. `status.scene` says why.

Still to do:
- Boxes outside the overview view, which need another calibrated camera view.
- Re-checking the goal right before release.
- A cube moved after detection (the cycle currently detects right before approach).
