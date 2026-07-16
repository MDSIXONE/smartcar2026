# Add local lidar isolated-return filtering

## Purpose

Suppress isolated laser returns that appear as noise in the local obstacle
layer, without changing the scan-to-map localization input or the global
static-wall filter.

## Changed files

- `ucar_ws/src/yolo2025/launch/2026.launch`
  - starts `jie_ware/lidar_filter_node` with `/scan` as the source,
    `/scan_filtered` as the output, and a `0.10 m` outlier threshold.
- `ucar_ws/src/ucar_nav/config/omni_test20250620/costmap_common_params.yaml`
  - makes the local obstacle layer consume `/scan_filtered`.
- `docs/operations.md`
  - documents the topic separation, deployment prerequisite, and runtime
    checks.

## Scope and safeguards

- `lidar_loc` continues to consume raw `/scan`; this preserves the verified
  `-sin(angle)` scan-to-map convention and avoids mixing this noise fix with
  localization behaviour.
- The global obstacle layer continues to consume `/scan_global_obstacles`.
- The filter only invalidates a finite range when both adjacent beams differ
  from it by more than `0.10 m`; it preserves the scan header, angles, and all
  retained range values.
- No map, global costmap, IMU, or CymPlanner/PD parameter was changed.

## Deployment and runtime verification

- Uploaded `2026.launch` and `costmap_common_params.yaml` to the vehicle; the
  deployed SHA-256 values match the local files.
- Restarted only `yolo2025 2026.launch startup_goal_enabled:=false`, so no
  default navigation goal or velocity command was sent.
- Confirmed `/lidar_filter_node` is running, publishes `/scan_filtered`, and
  `/move_base` subscribes to that topic.  The filtered scan published at about
  `12 Hz`, matching the raw scan rate.
- Across 36 stationary raw and filtered scans, the mean finite-return count
  changed from `261.69` to `256.39`: an average of `5.31` isolated returns per
  scan was removed.  Retained ranges are not modified by this filter.
- Confirmed `/odom_raw`, `odom -> base_link`, and `map -> base_link` contain
  finite values before any navigation test.
- The YDLidar driver reuses `LaserScan.header.seq` across distinct timestamps;
  the runtime check therefore uses multi-frame statistics rather than treating
  that sequence number as a unique scan identifier.

## Verification plan

1. Validate the launch XML and costmap YAML locally.
2. Restart the no-goal navigation launch after deploying the two changed
   configuration files.
3. Confirm `/scan_filtered` publishes near the raw `/scan` rate and compare
   both topics in RViz while stationary.
4. Confirm the local costmap no longer marks isolated returns, then perform a
   low-risk navigation check only after localization and TF are finite.

## Known limitation

This single-frame neighbour filter is designed for isolated one-beam noise. It
does not remove broad reflective patches, moving objects, or errors in TF,
odometry, and scan-to-map alignment.
