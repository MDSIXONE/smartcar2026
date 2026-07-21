# 2026-07-21 Port simulation CymPlanner flow to RViz-only UCar navigation

## Purpose

Use the control and obstacle-response flow from the Noetic simulation for the
Melodic UCar workspace, while reducing the live entrypoint to manual RViz
goals only.  The launch no longer starts a default goal, QR scanning, speech,
camera, or production-route logic.

## Changed files

- `ucar_ws/src/yolo2025/launch/2026.launch`: keeps only bringup, map server,
  laser localisation, scan filtering, CymPlanner move_base, and RViz visual
  model setup.
- `ucar_ws/src/yolo2025/scripts/navigation_scan_relay.py` and
  `CMakeLists.txt`: split the required `/scan_raw -> /scan` bridge and
  static-wall-filtered global obstacle stream out of the old task script.
- `ucar_ws/src/yolo2025/launch/2026.launch`: makes the scan relay retry after
  `2.0 s` if a transient startup failure occurs, so it cannot leave
  localisation permanently without `/scan`; it also restores only the USB
  camera publisher for RViz, without reviving QR/task logic.
- `ucar_ws/src/cym_planner/`: adds the simulation's latched lookahead
  footprint Marker and its Melodic `visualization_msgs` dependencies.
- `ucar_ws/src/ucar_nav/config/testnav20260721/`: is the active named
  navigation profile; it separates global and local
  costmap common files and synchronises their simulation update rates,
  resolutions, obstacle/raytrace ranges, layer order, and inflation tuning;
  the real vehicle's local rolling window is intentionally compact at
  `2.0 m x 2.0 m`; its global inflation radius is set to `0.205 m`.
- `ucar_ws/src/ucar_nav/launch/cym_move_base_omni_2026.launch`: loads every
  costmap, GlobalPlanner, and move_base YAML from `testnav20260721` rather
  than the retained historical `omni_test20250620` directory.
- `ucar_ws/src/ucar_nav/maps/iflysse_field_walls_without_middle_vertices.*`:
  stores the real-vehicle map without middle vertices, which is the current
  default map_server input.
- `ucar_ws/src/yolo2025/rviz/navigation_2026.rviz`: displays the CymPlanner
  lookahead footprint instead of the removed task marker, and adds the real
  `/usb_cam/image_raw` Image panel.
- `docs/quickstart.md` and `docs/operations.md`: record the RViz-only flow,
  build, launch, verification, and safety limits.
- `.gitignore`: excludes the root `simulation/` workspace from the
  vehicle-only `simulation_real` branch.

## Environment adaptation

The simulation's Gazebo and `base_footprint` interfaces are intentionally not
copied to the vehicle.  The UCar keeps ROS Melodic/Python 2, `base_link`,
laser-localisation `map -> odom`, raw `/scan` for localisation, filtered
`/scan_filtered` for local avoidance, and the measured `0.342 m x 0.256 m`
footprint.  The global scan filter is retained because the real laser and
static map have a known small alignment offset.  Thus the remaining global
and local costmap tuning follows the simulation, while the physical footprint,
frames, and laser interfaces safely remain vehicle-specific.

The real vehicle also uses a `2.0 m x 2.0 m` local rolling costmap instead of
the simulation's `5.0 m x 5.0 m` window.  This is a user-requested near-field
avoidance limit; the local obstacle range remains `3.0 m`, so returns outside
the rolling map are naturally clipped rather than changing sensor semantics.

The global inflation radius is `0.205 m` (cost scaling factor `0.05`).  The
default map is `iflysse_field_walls_without_middle_vertices.yaml`, rather than
the prior `iflysse_2026_direct.yaml` map.

## Verification

- XML parsing passes for `2026.launch` and `cym_planner/package.xml`; YAML
  parsing passes for the CymPlanner and move_base configurations; Python syntax
  parsing passes for `navigation_scan_relay.py`; and `git diff --check` passes.
- An isolated WSL Ubuntu 20.04 / ROS Noetic catkin workspace builds
  `libcym_planner.so` with the added `visualization_msgs` dependency and runs
  the final-yaw suite successfully: `4 tests, 0 errors, 0 failures, 0 skipped`.
- `roslaunch --nodes yolo2025 2026.launch` parses against the source workspace
  without starting hardware nodes and reports only the intended 12 nodes:
  driver, odometry, lidar, scan relay, map/localisation/filter, move_base, and
  pose publisher.
- On the stationary Melodic vehicle, the first run showed the laser timestamps
  occasionally leading `map <- laser_frame` TF by a few milliseconds.  The
  relay now accepts a latest common TF no older than `0.20 s`; it remains
  fail-closed for missing or stale localisation.
- After the relay adjustment, the real Ubuntu 18.04 / ROS Melodic workspace
  rebuilt `cym_planner` and `yolo2025` successfully.  The no-goal launch has
  one `/cmd_vel` publisher (`/move_base`), finite stationary `/odom_raw`, both
  required TF links, and a live global obstacle scan (`46/336` finite beams in
  the sampled frame).  No RViz goal or non-zero motion command was sent.
- On 2026-07-21 the no-goal navigation launch was explicitly interrupted after
  a zero `/cmd_vel` command before costmap changes were made.  The updated
  global/local common YAML files and the two costmap YAML files pass local YAML
  parsing; the updated launch passes XML parsing and `git diff --check`.
- The five costmap/launch files were synchronised to the vehicle with matching
  SHA-256 values.  On the vehicle, `roslaunch --nodes yolo2025 2026.launch`
  resolved the intended 13 nodes without launching them.  The navigation node
  list was empty afterward; the task remains stopped.
- The vehicle local costmap configuration was subsequently reduced to
  `width: 2.0` and `height: 2.0`.  Local Noetic YAML parsing and vehicle
  Melodic/Python 2 YAML parsing both asserted those exact values; no navigation
  node was restarted for this change.
- The scan relay retry change passes launch XML parsing and vehicle
  `roslaunch --nodes` parsing.  It was deployed and launched with no goal:
  relay, map server, lidar localisation/filtering, and move_base remained
  registered; `/scan` was live; `/odom_raw` was finite with zero twist; and
  both `odom -> base_link` and `map -> base_link` were available.  No RViz
  goal or non-zero motion command was sent.
- Camera restoration passes launch XML/RViz YAML validation and vehicle static
  launch parsing.  On the vehicle, `/usb_cam` publishes
  `/usb_cam/image_raw` at approximately `30 Hz`; the current-project RViz
  process is subscribed to that topic.  The USB stream is independent of
  `/cmd_vel`, QR, speech, and task nodes.
- The no-middle-vertices YAML/PGM resources were copied from the vehicle into
  the local workspace, validated as a matching pair, and deployed back from
  the workspace.  The global-costmap YAML and launch XML parse successfully;
  the running vehicle reports global inflation `0.205`, local size `2 x 2`,
  finite stationary odometry, and a live `map -> base_link` transform.  Its
  actual map_server command uses `iflysse_field_walls_without_middle_vertices.yaml`.
- `testnav20260721` was created as the named active profile, all six of its
  YAML files passed local Noetic and vehicle Melodic/Python 2 YAML parsing, and
  the vehicle launch was restarted at zero speed.  The running node reports
  global inflation `0.205`, local size `2 x 2`, finite stationary `/odom_raw`,
  and a live `map -> base_link` transform.
- The real-vehicle changes were committed on `simulation_real`
  (`55af7cf`).  A follow-up commit (`c910537`) removes the inherited root
  `simulation/` workspace from that branch and ignores it for future commits.
  The local workspace was intentionally not retained; existing remote branches
  such as `size-cymplanner` were not altered.

## Known limitations

The lookahead Marker is published only after move_base receives a path.  A
vehicle must not receive an RViz goal until `/odom_raw` is finite and both
`odom -> base_link` and `map -> base_link` transforms are available.  The
required runtime files have been transferred to the vehicle and validated only
while stationary.  The source-only changes are committed and pushed on
`simulation_real`; the root `simulation/` workspace is not part of that branch.

## Recorded issue: 2026-07-21 missing `map -> base_link` after full launch

During a no-goal full launch, move_base repeatedly reported that `map` was not
available while building its costmaps.  The immediate cause was
`navigation_scan_relay` exiting with code 1 about two seconds after startup.
That removes the `/scan_raw -> /scan` bridge; without `/scan`, `lidar_loc`
cannot publish `map -> odom`, so `map -> base_link` cannot exist.  This is not
a costmap-tuning fault.

The launch was stopped with zero velocity, and no navigation nodes remain
active.  The relay has been checked as an executable LF Python 2 script; its
ROS imports and Python compilation pass, and a standalone diagnostic run kept
the node alive.  The full-launch process did not persist the relay's pre-ROS
stderr, so the specific exit exception remains unconfirmed.  On recurrence,
capture the terminal output around `[navigation_scan_relay-8] process has
died` before changing costmaps or sending a goal.
