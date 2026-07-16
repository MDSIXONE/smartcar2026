# RViz navigation default view

## Purpose

Make the WSL RViz launcher consistently open a navigation-focused view with
the task map, both move_base costmaps, live lidar points, and a non-stale
vehicle-heading indication.

## Changed files

- `ucar_ws/src/yolo2025/rviz/navigation_2026.rviz`
  - keeps `/map`, `/move_base/global_costmap/costmap`, and
    `/move_base/local_costmap/costmap` enabled;
  - labels the `/scan` display as lidar points and limits its visual trail to
    0.3 seconds;
  - shows only the `base_link` TF heading arrow and hides it when its TF has
    been stale for 0.3 seconds.
- WSL `/root/start_rviz.sh`
  - opens the project RViz configuration with `rviz -d` by default.
- `docs/operations.md`
  - records the default displays and 0.3-second freshness behavior.

## Verification

- The RViz configuration was checked for the expected fixed frame (`map`),
  topics, enabled displays, and 0.3-second timeout/decay values.
- The WSL launcher checks that the project configuration exists before it
  invokes RViz.
- Restarting `~/start_rviz.sh` opened a window titled
  `navigation_2026.rviz - RViz`; its Displays panel shows all configured
  navigation items enabled.
- The vehicle was then started with `startup_goal_enabled:=false` against the
  WSL Master. `/map`, both costmaps, `/scan`, `/odom`, and `map -> base_link`
  all delivered data to WSL; the observed `/scan` rate was about 12 Hz.

## Known limitation

The displays remain empty until the vehicle has joined the WSL ROS Master and
is publishing the corresponding navigation topics and TF transforms.
