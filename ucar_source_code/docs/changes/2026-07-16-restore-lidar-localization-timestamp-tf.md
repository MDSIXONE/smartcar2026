# Restore lidar localization with scan-timestamp TF correction

## Purpose

AMCL drifted during longer runs because this chassis cannot provide sufficiently
reliable odometry for its motion model.  Restore the proven scan-to-map matcher
and prevent the visible local-costmap/laser rotation caused by mixing a matched
scan pose with a newer odometry pose.

## Changed files

- `ucar_ws/src/yolo2025/launch/2026.launch`
  - removes AMCL and starts `jie_ware/lidar_loc` as the sole `map -> odom`
    publisher.
- `ucar_ws/src/jie_ware/src/lidar_loc.cpp`
  - records the timestamp of every matched scan;
  - queries `odom -> base_link` at that exact timestamp instead of `Time(0)`;
  - timestamps the correction slightly into the future, and publishes it
    immediately after processing the scan.
  - predicts the next scan-match seed from the bounded odometry increment,
    then applies the existing laser-based refinement.  Invalid/implausible
    wheel-odometry jumps are ignored rather than moving the global pose.
- `ucar_ws/src/ucar_nav/launch/cym_move_base_omni_2026.launch`
  - overrides only the live local-costmap `transform_tolerance` to `0.10 s`;
    the vehicle's user-edited YAML files are not uploaded or replaced.
- `docs/operations.md`
  - replaces the obsolete AMCL deployment commands with the lidar-localization
    build, deployment, and verification commands.

## Verification plan

- Parse the edited launch XML and compile `jie_ware` on the vehicle.
- Start with `startup_goal_enabled:=false`; confirm `/lidar_loc` exists and
  `/amcl` does not.
- Confirm `map -> odom -> base_link` is live while stationary, then perform a
  supervised in-place rotation.  The laser display and the local costmap must
  remain fixed to the static map rather than rotate with `base_link`.

## Verification result

- The two edited launch files parsed as XML locally, and the vehicle completed
  `catkin_make -DCATKIN_WHITELIST_PACKAGES="jie_ware" --pkg jie_ware` without
  compiler errors.
- The vehicle is running the no-goal launch with `/lidar_loc` present,
  `/amcl` absent, `/scan` at about 12.6 Hz, and runtime
  `/move_base/local_costmap/transform_tolerance` equal to `0.1`.
- A first no-translation rotation probe isolated the former seeding lag: its
  static-wall overlap fell from 100% to 59% during a roughly 51 degree turn.
- After deploying the bounded odometry-prediction change, `/odom_raw` briefly
  published `NaN`; the launch was stopped and restarted before any motion.
  The recovered pipeline published finite odometry and live `odom ->
  base_link` and `map -> base_link` transforms.
- A second probe then sent no navigation goal and made one clockwise command
  only (`angular.z=-0.20` for 3.58 seconds); it did not turn back.  The
  measured `map -> laser_frame` heading changed from 0.025 to 0.606 rad and
  the static-wall overlap was 29.5% before versus 24.4% immediately after
  stopping.  No new TF/odometry warnings appeared after that rotation.

## Known limitation

The scan matcher needs a valid map, `/scan`, and fixed `base_link ->
laser_frame` transform.  A `wheelodom` or `/odom_raw` `NaN` invalidates all
localization checks: stop the chassis and restart the odometry/navigation
chain before testing again.  Rotation verification uses the relative change
in wall overlap; the absolute ratio depends on the current physical placement
and map-cell tolerance.  No automatic navigation goal is sent during that
verification.
