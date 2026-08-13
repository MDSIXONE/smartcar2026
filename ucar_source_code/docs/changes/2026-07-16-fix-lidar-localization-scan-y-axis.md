# Fix lidar localization scan Y axis

## Purpose

Align laser localization with the ROS `LaserScan` convention and with maps
created by gmapping, preventing a persistent clockwise map-to-odom rotation
in RViz.

## Changed files

- `ucar_ws/src/jie_ware/src/lidar_loc.cpp`
  - changes each scan point from mirrored Y (`-r*sin(angle)`) to the standard
    ROS left-positive Y (`r*sin(angle)`) before the laser-to-base TF and map
    matcher are applied.
- `docs/operations.md`
  - documents the scoped deployment, build, and no-motion restart procedure.

## Verification

- The live scan reports a positive `angle_increment` over `[-pi, pi]`, so its
  points use the standard ROS right-handed LaserScan convention.
- The navigation map was produced by gmapping from the same `/scan` topic and
  `base_link -> laser_frame` transform.
- Before this fix, `map -> odom` was measured at approximately `-8.73 degrees`
  while the vehicle was stationary, matching the visible clockwise offset.
- The corrected source was checksum-verified after upload, and
  `catkin_make --pkg jie_ware` completed successfully on the vehicle.
- After a no-motion restart (`startup_goal_enabled=false`), `map -> odom`
  measured about `-0.13 degrees`; the prior persistent clockwise rotation was
  removed.

## Known limitation

This change was superseded by the wall-overlap regression recorded in
`2026-07-16-restore-lidar-scan-y-sign.md`.  The nominal ROS `LaserScan`
convention alone was not sufficient for this vehicle's YDLidar-to-OpenCV-map
pipeline: the task map and matcher require the original mirrored Y input.

After restart, a correct initial pose is still required. Use RViz **2D Pose
Estimate** if the physical vehicle was not placed exactly at the configured
start pose.
