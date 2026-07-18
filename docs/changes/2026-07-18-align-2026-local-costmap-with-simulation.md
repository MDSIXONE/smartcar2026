# 2026-07-18 Align 2026 local costmap with simulation

## Purpose

Align the Task 2026 local navigation footprint and numerical local-costmap
tuning with the active Task 3 simulation configuration.

## Changed files

- `ucar_ws/src/yolo2025/scripts/2026.py`
- `ucar_ws/src/yolo2025/launch/2026.launch`
- `ucar_ws/src/ucar_nav/config/omni_test20250620/local_costmap_params.yaml`
- `ucar_ws/src/ucar_nav/launch/cym_move_base_omni_2026.launch`
- `docs/operations.md`

## Aligned values

- Footprint: `[[0.18, -0.12], [0.18, 0.12], [-0.18, 0.12], [-0.18, -0.12]]`
  (`0.36 m × 0.24 m`).
- Local rolling window: `5.0 m × 5.0 m`, `0.03 m` resolution.
- Update/publish rates: `12 Hz` / `5 Hz`.
- Obstacle/raytrace ranges: `3.0 m` / `4.0 m`.
- Inflation: `0.07 m` radius and `4.0` cost-scaling factor.

## Intentional real-vehicle interfaces retained

The real vehicle keeps `global_frame: map`, `robot_base_frame: base_link`, and
the `/scan_filtered` local observation topic.  The simulation's
`odom` / `base_footprint` / `/scan` interfaces are not interchangeable with
the real lidar localization and filtered-scan pipeline.

## Verification

- `2026.py` is Python-syntax checked.
- The launch and YAML files are statically checked for the aligned footprint
  and local-costmap values.

## Deployment

Copy `2026.py`, `2026.launch`,
`cym_move_base_omni_2026.launch`, and `local_costmap_params.yaml` to the
vehicle.  Rebuild `yolo2025` if necessary, then stop the previous navigation
launch and restart with `startup_goal_enabled:=false` before any movement test.
