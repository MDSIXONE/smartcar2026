# Add the post-QR production-grid centre route

## Purpose

Replace the old three-goal post-QR route with a `move_base`-planned,
production-grid-centre sequence `2 -> 12 -> 22 -> 32 -> 31 -> 21 -> 11 -> 1`.
Every diagonal request is expanded through a valid grid centre so planned route
goals never omit an intentional corner.

## Changed files

- `ucar_ws/src/yolo2025/scripts/2026.py`
  - retains `move_base` for every production-route segment and checks that
    `/move_base/make_plan` returns a nonempty path before sending each goal.
  - expands diagonal requests horizontally then vertically through a numbered
    centre: for example `1 -> 26` becomes `1 -> 6 -> 26`, where 6 has point
    1's row and point 26's column.
  - sends each grid goal with its approach yaw, then uses a bounded in-place
    alignment state at the reached centre before planning the next segment.
    The alignment commands only `angular.z`, have a `0.07 rad` completion
    tolerance, and stop the task after `8 s` rather than rotating indefinitely.
  - limits CymPlanner task speed to `0.25 m/s`, keeps its holonomic mode false,
    and keeps the CymPlanner `0.05 m` motion-completion threshold. The
    post-action localisation audit allows `0.08 m`, because lidar localisation
    can refresh by a few millimetres after the action result.
  - disables only the local dynamic `ObstacleLayer` during the production
    stage. The global layer remains enabled and its scan source is frozen, so
    its existing cost contribution remains while no new laser marks or clears
    are supplied. No production-stage path calls `clear_costmaps`.
  - publishes a latched RViz footprint marker in `base_link`: red is the
    physical `0.342 × 0.256 m` chassis outline and yellow is its `0.05 m`
    safety envelope.
- `ucar_ws/src/yolo2025/launch/2026.launch`
  - replaces the legacy three post-QR targets with the selected route and
    grid-goal, bounded-alignment, and footprint parameters.
- `ucar_ws/src/yolo2025/config/production_square_centers.json`
  - packages the supplied 40 production-square centres for vehicle deployment.
- `ucar_ws/src/yolo2025/CMakeLists.txt` and `package.xml`
  - install the Python task script and production-centre resource and declare
    the `dynamic_reconfigure`, `std_srvs`, and `visualization_msgs` runtime
    dependencies.
- `ucar_ws/src/yolo2025/rviz/navigation_2026.rviz`
  - displays the latched footprint/safety-margin marker alongside the map,
    costmaps, laser scan, and base-link arrow.
- `docs/operations.md`
  - documents production-route behavior and deployment commands.

## Validation

- The packaged 40-centre JSON exactly matches the supplied point list.
- The configured route resolves to the requested coordinate sequence:
  `(-1.75, 1.25)`, `(-1.75, 0.75)`, `(-1.75, 0.25)`,
  `(-1.75, -0.25)`, `(-2.25, -0.25)`, `(-2.25, 0.25)`,
  `(-2.25, 0.75)`, `(-2.25, 1.25)`.
- Diagnosis evidence: the former code cancelled `move_base`, disabled both
  obstacle layers, and started the direct controller with waypoint index 0 at
  the QR stop.  The first direct segment therefore crossed the map boundary
  toward point 2 and could collide with its wall.
- The corrected state machine never publishes driving `cmd_vel`; it checks a
  global plan then sends the first grid goal to `move_base`, avoiding the wall
  collision caused by the former direct QR-stop-to-2 line.
- Route-expansion logic resolves the supplied `1 -> 26` example to exactly
  `[1, 6, 26]`; the configured route stays `[2, 12, 22, 32, 31, 21, 11, 1]`
  because every selected pair is already in one row or column.
- Regression harness: a successful point-2 goal followed by the observed
  `0.057 m` post-action map pose failed before this change and advances to
  point 12 with the `0.08 m` audit guard. A pose above `0.08 m` still stops the
  task.
- Diagnosis evidence: the previous production entry both disabled the global
  `ObstacleLayer` and called `/move_base/clear_costmaps`; this removed existing
  cost contributions instead of merely preventing new ones. The revised entry
  only disables the local layer and freezes publication to
  `/scan_global_obstacles`.
- Diagnosis evidence: the old 31 goal was sent with yaw `+pi/2` (the heading
  for 31 -> 21), and no `point=31 reached` event followed. The revised route
  reaches 31 with its westward approach yaw, then makes the required northward
  in-place alignment at 31 under an eight-second safety deadline.
- Python syntax and launch/package XML validation pass locally.
- The vehicle ROS Melodic environment imports `dynamic_reconfigure.client` and
  `std_srvs.srv.Empty` successfully; the new deployment additionally requires
  the standard Melodic `visualization_msgs` package.
- The vehicle workspace may retain a `jie_ware`-only catkin whitelist; build
  this update with `-DCATKIN_WHITELIST_PACKAGES="jie_ware;yolo2025"` so the
  Python package's install and resource rules are generated.
- The five deployed source resources were SHA-256 matched on the vehicle;
  `2026.py` is executable (`0755`). The remote
  `catkin_make -DCATKIN_WHITELIST_PACKAGES="jie_ware;yolo2025"` build and
  `roslaunch --nodes yolo2025 2026.launch` static node expansion both passed.
- No `2026.launch` process is running after deployment.

## Known limitation

- This update intentionally disables local dynamic lidar avoidance during the
  production stage and freezes the global obstacle input. Existing global
  costs remain, but the fixed grid segments must be physically clear before
  the QR sequence enters the grid.
- No motion test is performed during deployment; the user starts the launch.
