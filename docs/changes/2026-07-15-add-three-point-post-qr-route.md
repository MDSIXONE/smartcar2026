# 2026-07-15 Add Three-Point Post-QR Route

## Purpose

Replace the previous two-point post-QR route with a three-point task route and limit only those three translational stages to `0.1 m/s`.

## Route

1. `map (-1.134, 1.505, yaw=pi)` in normal mode.
2. The previous relative goal: from the first point's actual `map -> base_link` pose, `(current_x, current_y - 1.2, yaw=pi)` in normal mode.
3. The previous fixed goal: `map (-2.134, -0.095, yaw=pi/2)` in holonomic mode; final rotation to 90 degrees starts only after position arrival.

## Changed files

- `ucar_ws/src/yolo2025/scripts/2026.py`
  - Implements the three-stage post-QR route and restores normal mode and the ordinary speed cap on completion or failure.
- `ucar_ws/src/yolo2025/launch/2026.launch`
  - Defines the three targets and `task_linear_speed=0.1`.
- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
  - Reads task-scoped `task_max_vel` for every new plan and caps `linear.x`/`linear.y` without changing ordinary navigation speed limits.
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - Adds a disabled-by-default `task_max_vel: 0.0` parameter.
- `docs/operations.md`
  - Documents the route, speed scope, and mode sequence.

## Verification

- Local Python syntax, launch XML, CymPlanner YAML, and C++ static-structure validation passed.
- Deployed the task script, task launch file, CymPlanner source, and CymPlanner configuration to the vehicle.
- Vehicle-side `catkin_make --pkg cym_planner`, Python 2 compilation, and `roslaunch --nodes yolo2025 2026.launch` validation passed.

## Scope and limitation

- The `0.1 m/s` cap applies only to translational commands in the three post-QR stages; existing angular speed limits remain unchanged.
- Navigation has not been started after this deployment.
