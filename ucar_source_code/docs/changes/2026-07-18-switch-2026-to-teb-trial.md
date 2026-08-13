# 2026-07-18 Switch 2026 navigation to TEB trial

## Purpose

Try the installed `teb_local_planner/TebLocalPlannerROS` on the real-vehicle
2026 navigation task to compare its terminal rotation and footprint handling
with the custom CymPlanner.

## Baseline

The CymPlanner final-yaw fix and all preceding simulation/RViz/local-costmap
changes were backed up to GitHub first in commit `aa58c5b` on
`MDSIXONE/smartcar2026`.

## Changed files

- `ucar_ws/src/ucar_nav/launch/teb_move_base_omni_2026.launch`
- `ucar_ws/src/ucar_nav/config/omni_test20250620/teb_move_base_params_2026.yaml`
- `ucar_ws/src/yolo2025/launch/2026.launch`
- `docs/operations.md`

## Change

- 2026 now includes a dedicated TEB move_base launch.
- Existing shared costmap, global planner, move_base, and
  `teb_local_planner_params.yaml` settings are loaded.
- A final override selects `teb_local_planner/TebLocalPlannerROS`.
- No `/cmd_vel` remap is used; TEB must publish directly to the base driver.
- `cym_move_base_omni_2026.launch` is unchanged and remains the rollback path.

## Availability and verification

- The vehicle's ROS Melodic environment contains
  `/opt/ros/melodic/share/teb_local_planner`.
- The local WSL Noetic environment does not contain the TEB plugin, so a local
  runtime test cannot load this planner. The vehicle must perform the final
  launch validation.
- Static XML parsing passed. The first vehicle launch attempt exposed the
  existing `ucar-mini` self-XML-RPC resolution problem; restarting with
  `unset ROS_HOSTNAME` and `ROS_IP=192.168.8.231` fixed it.
- With `startup_goal_enabled:=false`, the vehicle now has `/move_base` and
  `/navigation_2026` running. `/move_base/base_local_planner` resolves to
  `teb_local_planner/TebLocalPlannerROS`, and
  `/move_base/TebLocalPlannerROS/max_vel_theta` is `0.8`.
- No navigation goal was sent. `/odom_raw` was finite and `odom -> base_link`
  was available during the no-motion check.

## Safety

Keep `startup_goal_enabled:=false` for the first trial. Confirm finite
`/odom_raw`, valid `odom -> base_link` and `map -> base_link` TF, and zero
velocity before sending a manual goal. If the trial is unsuitable, restore the
CymPlanner include and restart navigation.
