# 2026-07-15 Enable Holonomic Post-QR Goal

## Purpose

Keep the robot at its 180-degree heading while it moves from the post-QR goal to the next fixed task point, then rotate to the requested 90-degree final heading only after reaching that point.

## Changed files

- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
  - Adds holonomic X/Y position control using target error transformed into `base_link`.
  - Suppresses `angular.z` during holonomic translation and performs final orientation alignment only after the position tolerance is reached.
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - Keeps `holonomic_mode=false` by default and adds `linear_y_gain=1.5`, `linear_y_kd=0.0`, and `max_vel_y=1.0`.
- `ucar_ws/src/yolo2025/scripts/2026.py`
  - After the post-QR relative goal reaches 180 degrees, switches CymPlanner to holonomic mode and sends fixed target `(-2.134, -0.095, yaw=pi/2)`; restores normal mode when it completes or fails.
- `ucar_ws/src/yolo2025/launch/2026.launch`
  - Adds parameters for the fixed next goal and documents why AMCL-only `odom_model_type` is not added to `lidar_loc`.
- `docs/operations.md`
  - Documents behavior and the required CymPlanner rebuild command.

## Verification

- Local Python syntax validation passed.
- Local launch XML and CymPlanner YAML validation passed.
- CymPlanner C++ brace-balance/static structure check passed. ROS Melodic headers and catkin are not available on this Windows workspace, so the required package build remains a vehicle-side deployment verification.
- Confirmed `ucar_controller/src/base_driver.cpp` consumes both `/cmd_vel.linear.x` and `/cmd_vel.linear.y`.
- After adding stage-based mode switching, repeated the local Python, launch XML, CymPlanner YAML, and C++ static-structure checks successfully.

## Scope and limitation

- This change is local only. It has not been built, uploaded, or run on the vehicle.
- `odom_model_type=omni` is intentionally not applied: the active localizer is `jie_ware/lidar_loc`, not AMCL. The base driver already accepts `linear.y`; holonomic behavior is provided by CymPlanner command generation.
- Startup navigation, QR heading changes, and the post-QR `y-1.2 m` target explicitly run in normal mode; only the fixed next goal runs in holonomic mode.
