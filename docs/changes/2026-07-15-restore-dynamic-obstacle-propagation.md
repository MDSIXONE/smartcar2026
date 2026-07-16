# 2026-07-15 Restore Dynamic Obstacle Propagation

## Purpose

Make laser-detected obstacles affect both immediate collision checking and global replanning, and switch to holonomic motion after the first post-QR target.

## Diagnosis

- The original standalone CymPlanner already rejected a blocked path segment and requested move_base recovery/replanning. The current planner retains this logic; it was not lost.
- The original planner's default collision lookahead was `0.8 m`; the current configuration limited it to `0.25 m` while the local costmap covered only `0.5 m × 0.5 m`.
- The active global costmap loaded only the static and inflation layers, so laser-observed obstacles were absent from the global planner.

## Changed files

- `ucar_ws/src/yolo2025/scripts/2026.py`
  - Switches to holonomic mode before sending the second post-QR goal; the first goal remains normal mode.
  - Before the automatic startup goal, requires a short stable `map -> base_link`
    interval and five consecutive valid global plans.
  - Publishes `/scan_global_obstacles`: laser returns whose endpoints are within
    `0.22 m` of a static-map wall (or outside the static-map bounds) are removed only from this global-planning
    topic. `/scan` remains unchanged for localization and the local costmap.
- `ucar_ws/src/yolo2025/launch/2026.launch`
  - Delays the initial startup-goal check for 15 seconds so lidar/odom and
    costmaps are available before stability is evaluated, then requires five
    valid planning samples before sending the goal.
- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - Restores `obstacle_lookahead_distance: 0.8`.
- `ucar_ws/src/ucar_nav/config/omni_test20250620/global_costmap_params.yaml`
  - Adds `obstacle_layer` to the global costmap and raises its update rate to `3 Hz`.
  - Uses a 2D `ObstacleLayer` sourced exclusively from
    `/scan_global_obstacles`, followed by normal `0.21 m` inflation. New laser
    obstacles still enter the global map, while static walls are not duplicated.
- `ucar_ws/src/ucar_nav/config/omni_test20250620/local_costmap_params.yaml`
  - Raises update frequency to `8 Hz` and expands the rolling window to `1.8 m × 1.8 m`.
- `ucar_ws/src/ucar_nav/config/omni_test20250620/costmap_common_params.yaml`
  - Extends laser obstacle range to `2.5 m` and retains observations for `0.6 s`.
- `ucar_ws/src/ucar_nav/config/omni_test20250620/move_base_params.yaml`
  - Raises global replanning frequency to `3 Hz`.
- `docs/operations.md`
  - Documents deployment and restart commands.

## Verification

- Local Python syntax, launch XML, and all affected YAML files validated successfully.
- Uploaded the task script and the five navigation/CymPlanner configuration files to the vehicle.
- Vehicle-side Python 2 compilation and `roslaunch --nodes yolo2025 2026.launch` validation passed.
- Verified on the vehicle that the deployed files contain the global laser
  obstacle layer, `3 Hz` global costmap/replanning, `8 Hz` local update,
  `1.8 m` local window, `2.5 m` obstacle range, `0.6 s` persistence, and
  `0.8 m` collision lookahead.
- On the vehicle, with the laser obstacle layer disabled and cleared, the default
  start-to-goal plan had `0` poses at `0.22 m` inflation but `1651` poses at
  `0.20 m`.
- With the global laser obstacle layer enabled, the same plan had `0` poses at
  `0.20 m`, `0.14 m`, and `0.12 m`, but `1265` poses at `0.10 m`. RViz also
  shows a slight laser/static-map offset; their separately inflated wall bands
  close the narrow doorway at the larger values.
- With the 2D global obstacle layer, static-only planning at `0.21 m` returned
  `2444` poses, but the unfiltered laser topic still returned `0` poses. The
  static-wall-filtered global topic is the final correction.
- After deployment, the vehicle reported `255` valid raw laser returns and
  `11` global filtered returns; at `0.21 m` the default route returned `2104`
  poses with the global obstacle layer enabled. No wheel-odometry NaN occurred
  during this validation launch.
- The observed offset reached `0.20 m`, so the global static-wall mask is
  widened to `0.22 m` and also masks returns outside the map bounds. A costmap
  clear then restored the static `0.21 m` route with `2444` poses. The filter
  now publishes an all-clear global scan until that mask is ready, preventing
  raw static-wall returns from being left in the global obstacle layer at boot.
- Final clean-boot verification required no manual costmap clear: `256` raw
  returns became `0` global obstacle returns, the default route had `2444`
  poses, and wheel-odometry NaN count was `0`.
- Full vehicle validation completed successfully: the default goal reached,
  QR codes `d`, `a`, and `i` were all recognized, and all three post-QR goals
  reached. The final stage restored normal CymPlanner mode and cleared the
  task-specific speed limit.
- A normal launch initially failed because its 2-second startup goal used the
  transient initial localization pose; after localization settled, the same
  current-to-goal probe returned `793` poses. The startup check now guards
  against this race.

## Limitation

- The static map is not permanently rewritten. Dynamic laser obstacles are marked and cleared by the global obstacle layer as sensor observations change.
- The durable improvement is to align the laser and static map. Static-map
  filtering intentionally omits a genuinely new obstacle that is immediately
  adjacent to a wall from global replanning, but the unfiltered local costmap
  still sees it and enforces collision avoidance.
