# 2026-07-23 CymPlanner direct-lidar update

## Purpose

Update the real-vehicle local planner so velocity selection is checked directly against fresh
filtered lidar points instead of relying only on the costmap.

## Changed files

- `ucar_ws/src/cym_planner/src/cym_planner.cpp` and `include/cym_planner.h`: add direct
  `LaserScan` ingestion, footprint rollout, candidate scoring, stale-scan stopping, blocked-path
  replanning, goal-directed in-place turning, terminal-yaw scoring, and RViz diagnostics.
- `CMakeLists.txt` and `package.xml`: enable C++11 and add `sensor_msgs`.
- `config/ucar_cym_planner_params.yaml`: add direct-lidar rollout parameters while retaining
  real-vehicle speed limits.
- `ucar_ws/src/yolo2025/rviz/navigation_2026.rviz`: display converted laser points, candidate
  trajectories, and the selected trajectory.
- `README.md`, `config/README.md`, and `docs/operations.md`: document the current vehicle-only
  control and deployment flow.

## Vehicle adaptations

- Use `/scan_filtered` rather than the unfiltered localization scan.
- Keep `max_vel_x: 0.5`, `max_vel_theta: 1.0`, and `final_yaw_max_vel: 1.0`.
- Publish diagnostics under `/move_base/cym_planner/CymPlanner/`.
- Use `/ucar/carry_mode` for the optional speed-scaling input.

## Verification

- Isolated ROS Noetic build produced `libcym_planner.so`.
- Existing final-yaw test suite passed: 4 tests, 0 errors, 0 failures, 0 skipped.
- YAML, XML and RViz configuration parsing are checked separately before deployment.

## Limitations

The updated source has not been copied to the physical vehicle or exercised with wheel motion.
Before any motion test, verify fresh `/scan_filtered`, finite odometry, required TF, and zero
velocity at startup.
