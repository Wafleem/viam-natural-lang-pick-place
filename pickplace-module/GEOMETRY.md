# Geometry: how a cube is found, grabbed, turned and placed

All positions are millimetres in the Viam `world` frame (arm base at the origin, Z up). Angles are
degrees. Code references are in `src/pickplace/`.

## 1. Frames

| Frame | Meaning |
|---|---|
| `world` | Arm base. Table top is about z = −10 (cube bottom when resting) |
| TCP (`Vgripper`) | Suction cup tip: arm flange + 126 mm along the flange Z. Tool Z points **down** during the task |
| cube frame **C** | Origin at the centre of the cube's top face, Z up, X along one cube edge (`grasp_geometry.cube_top_frame`) |
| camera `cam` | Wrist RealSense, (−73, 40, 18) mm and −90° about Z on the arm flange |

A tool pose "pointing down with yaw ψ" is `downward_pose(xyz, ψ)`: orientation vector (0, 0, −1) and
θ = −ψ (wrapped). Rotating ψ by 90° turns the cup about the vertical and nothing else.

## 2. Seeing the cube (vision → world → tool coordinates)

1. At the **calibrated overview** joints (the only pose the calibration is valid for), five aligned
   RGB-D frames are captured (`vision_geometry.analyze`). Each cube's top face is fitted as a
   horizontal plane, giving its centre **p_cam** and its edge yaw **φ_cam** in [−45°, 45°).
2. Four of the five frames must agree within 5 mm and 3° (`consensus_observation`).
3. The overview calibration maps camera-world XY to the taught tool XY (`calibration.calibrated_xy`):

   **p = R(γ) · p_cam,xy + o**, with γ = `yaw_correction_deg` (−2.12°) and o = `xy_offset_mm`.

   Edge yaw: **φ = φ_cam + γ**. Heights are not taken from depth: the contact height is the taught
   `pickup_tcp_z_mm` (190.3), and depth only checks that the cube rests on the table.

## 3. Grabbing

A square cube looks the same every 90°, so the pickup yaw can be **ψ_pick = φ + 90k** for k = 0…3.
For each k, IK (`kinematics.ik`) is solved for the above-pick pose, seeded at the current joints.
Joint-limit margin is 5°. The k with the least total joint travel wins (`task.compute_targets`).
In practice this keeps the wrist on the taught branch (J4 ≈ 180°) instead of rolling to J4 = 360°,
which faulted the controller in 0.0.24.

The cup is not centred on the face. It sits `grasp_toward_image_top_mm` (15 mm) toward the cube edge
that faces the top of the camera image, away from the printed logo (`task.grasp_offset`).

Contact pose: **T_pick = downward_pose(p + offset, ψ_pick)** at z = 190.3.
Press pose: the same pose 10 mm lower (`press_depth_mm`), with suction on.

The **grasp transform** is the tool pose seen from the cube (`grasp_geometry.describe_grasp`):

**G = C_pick⁻¹ · T_pick**, where C_pick is the cube frame at pickup with yaw ψ_pick.

G is fixed for the whole cycle: the cube doesn't slide on the cup. With the 15 mm offset,
G's translation is (0, −15, 0).

## 4. Placing: same cube, new pose

The goal is a cube frame **C_goal**. Its top centre is the goal point, and its yaw is ψ_place.

- **Box:** top centre = (`placement_center_xy_mm`, floor z + cube size). ψ_place = box yaw + 90k
  (`world_yaw_deg`), so the cube's edges are parallel to the walls.
- **Ring:** top centre = calibrated ring XY + `placement_correction_world_xy_mm`, at z =
  `place_tcp_z_mm`. ψ_place = `placement_world_yaw_deg` + 90k.

Release pose: **T_release = C_goal · G** (`grasp_geometry.place_tcp`). The tool is placed wherever
puts *the cube*, not the tool, at the goal, including the 15 mm off-centre grip turned by
(ψ_place − ψ_pick). Of the four k, the one with the least joint travel from the lift pose wins.

Check (tests/test_geometry.py): **T_release · G⁻¹ = C_goal** to 1e-6 mm, and the cube yaw at
release equals the box yaw (mod 90°).

## 5. Will it fit? (`box.check_cube_fits`)

In the box frame (rotate by −box yaw about the floor centre), with δ the cube centre's offset and
θ = ψ_place − box yaw:

  extent = s · (|cos θ| + |sin θ|)          (= s when the cube is square to the walls)
  gap_i  = W_i/2 − |δ_i| − extent/2         (must be ≥ 0 on both axes)

For the current box (W = 261.4 × 206.6 mm, s = 200 mm) the gaps are **30.7 mm and 1.4 mm**. The
narrow side would have 3.3 mm per side if the cube were centred on the floor; the configured
`placement_center_xy_mm` sits 1.9 mm off-centre, which uses up most of that. A 5° yaw error would
add s·(cos5° + sin5° − 1) ≈ 16.7 mm of extent and not fit. Yaw alignment is therefore what matters
most, and vision measures yaw to about ±3°.

To centre the cube exactly, remove `placement_center_xy_mm` from the box entry; the floor centre is
then used. The configured point came from an operator-taught placement, so try both with `dry_run`.

## 6. Heights during the move

| Pose | Tool z |
|---|---|
| above pick | contact + `clearance_mm` (100) = 290 |
| pre-contact | contact + `pre_contact_mm` (20) |
| press | contact − `press_depth_mm` (10) |
| above a box | rim + cube + `box_entry_clearance_mm` (30) = 360: cube bottom 30 mm over the rim |
| release | goal top (cube bottom on the floor or ring) + `release_above_goal_mm` (0) |

While carried, the cube is modelled as a box attached to the TCP. It spans [−press, size] along tool
Z, so it covers both the compressed foam and the uncompressed cube. The planner checks it against
the table patch, the ring, every box wall and the workcell walls.

## 7. Adding a box

1. Put the box (two orange Post-its on it) where the overview camera sees all four walls and the
   inside floor. Dashboard → *Run* → reset, then *Vision* → `measure_box` with a name and colour.
2. The survey (`box_survey.measure_box`) fits the floor plane and four, possibly leaning, wall
   planes. The opening is the cavity common to the full wall height. Three frames must agree:
   3 mm, 3 mm and 1°.
3. The candidate is converted with the same overview calibration: XY by R(γ)·p + o, yaw + γ. Heights
   are shifted by the amount that maps the observed tabletop to the taught one
   (`box_survey.candidate_entry`).
4. Check the opening with a tape measure. Then press *Add candidate to config*, followed by
   `dry_run` with the new goal. The new box is a goal and an obstacle for every other goal.
5. Clearance rule of thumb: opening ≥ cube size + 2 × (expected position error), plus margin for
   yaw. For the 200 mm cube, an opening under about 210 mm on either axis is very tight.
