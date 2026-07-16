# 2026-07-15 Deploy Map and Holonomic Task Changes

## Purpose

Deploy the updated 2026 map, QR task sequence, stage-based holonomic navigation, and CymPlanner implementation to the vehicle.

## Deployed files

- `ucar_nav/maps/iflysse_2026_direct.pgm`
- `yolo2025/scripts/2026.py`
- `yolo2025/launch/2026.launch`
- `cym_planner/src/cym_planner.cpp`
- `cym_planner/config/ucar_cym_planner_params.yaml`

## Verification on vehicle

- Rebuilt `cym_planner`; `devel/lib/libcym_planner.so` was regenerated.
- Python 2 compilation of `2026.py` passed.
- `roslaunch --nodes yolo2025 2026.launch` passed and listed all expected nodes.
- Confirmed `holonomic_mode: false` is the startup default and the fixed next goal is `(-2.134, -0.095, 1.570796)`.

## Scope and limitation

- Navigation was not started after deployment.
- No global costmap parameters or unrelated control configuration files were changed.
