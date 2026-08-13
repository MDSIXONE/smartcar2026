# Restore lidar scan Y sign for the task map

## Purpose

Correct the scan-to-map coordinate convention after RViz showed laser points
rotating relative to static walls during an in-place turn.  The prior generic
ROS-convention change used `+r*sin(angle)`, but this YDLidar task map and its
OpenCV matcher require the original mirrored image-row coordinate.

## Changed files

- `ucar_ws/src/jie_ware/src/lidar_loc.cpp`
  - restores `y_laser = -range*sin(angle)` and documents that this is an
    internal matcher/map convention rather than a change to the ROS `/scan`
    message itself.
- `docs/operations.md`
  - records the hardware/map-specific scan-Y rule for future deployments.
- `docs/changes/2026-07-16-fix-lidar-localization-scan-y-axis.md`
  - marks the earlier generic-Y assumption as superseded.

## Evidence before deployment

- With the car stationary after the controlled turn, the running `+sin` source
  scored only 12/25 (48.0%) static-wall matches at the current TF pose.
- Evaluating the same scan and pose with the restored `-sin` convention scored
  21/27 (77.8%).  A yaw search reached 30/31 (96.8%) with that convention.

## Deployment verification

- Uploaded `ucar_ws/src/jie_ware/src/lidar_loc.cpp` to
  `ucar@192.168.8.231:~/ucar_ws/src/jie_ware/src/`.
- Confirmed the deployed source SHA-256 equals the local source:
  `e568291a0f3e14e77e3288712ada0fc774be0a61da9cdbf07c744649d33642a8`.
- Confirmed the deployed source retains
  `y_laser = -msg->ranges[i] * sin(angle)`.
- Built successfully on the vehicle with
  `catkin_make -DCATKIN_WHITELIST_PACKAGES=jie_ware --pkg jie_ware`;
  the resulting executable is
  `~/ucar_ws/devel/lib/jie_ware/lidar_loc` (modified 2026-07-16 18:01:10
  +0800).
- Navigation has not been restarted and no motion command was sent as part of
  this deployment.

## Verification plan

1. Build `jie_ware` on the vehicle and restart the no-goal launch.
2. At the physical start pose, verify stationary static-wall overlap before
   moving.
3. Perform one clockwise in-place turn without turning back.  Compare IMU,
   odometry, and `map -> laser_frame` yaw, and observe that the displayed scan
   remains fixed to static walls.

## Known limitation

The IMU/odometry yaw sign is opposite the map correction observed in the
current regression.  This change intentionally restores only the scan-Y
mapping variable.  The odometry-prediction yaw sign will be tested separately
after this regression so the two causes remain distinguishable.
