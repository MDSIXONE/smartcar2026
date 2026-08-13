# Switch 2026 task localization to AMCL

## Purpose

Replace the custom `lidar_loc` localization node.  Map/scan mismatches could
make its `map -> odom` correction jump during rotation, making laser returns
appear to rotate with the robot and causing false obstacle markings.

## Changed files

- Added `ucar_ws/src/ucar_nav/launch/amcl_2026.launch`: AMCL publishes
  `map -> odom`, consumes `/scan`, and uses the `omni` motion model.
- Set `update_min_d` and `update_min_a` to `0.0` so every scan is used after
  any reported motion; the former 5 cm / 0.05 rad gate allowed visible drift
  when the chassis under-reported short movements.
- Updated `ucar_ws/src/yolo2025/launch/2026.launch`: removed the active
  `jie_ware/lidar_loc` node and included the AMCL launch instead.
- Updated `docs/operations.md` with AMCL deployment and verification commands.

## Verification

- Parse both launch files as XML.
- Confirm that the 2026 task launch contains no active `lidar_loc` node.
- On the vehicle, start with `startup_goal_enabled:=false`, then confirm that
  `/amcl` exists, `/lidar_loc` does not, `odom_model_type` is `omni`, and the
  `map -> odom -> base_link` TF chain is live.
- Verified at runtime that the dynamic AMCL parameters accept `0.0` for both
  update thresholds; this does not command vehicle motion.
- With the vehicle placed at the configured start pose, ran one bounded
  0.20 m forward test at `0.10 m/s`, then automatically cancelled the goal
  after five seconds and sent zero velocity.  The target was
  `(-0.050, 2.750, yaw 0)` and the final AMCL estimate was
  `(-0.074, 2.747, yaw 0.088)`, a 2.4 cm translation error.

## Known limits

AMCL still requires a correct `odom -> base_link` transform, a fixed
`base_link -> laser_frame` transform, and a sensible initial pose.  Use RViz
**2D Pose Estimate** if the physical placement differs from the configured
initial pose; do not run `lidar_loc` alongside AMCL.

The bounded short test does not prove a long route is collision-free.  A
longer, physically supervised run must still verify that AMCL remains aligned
with the static map throughout the route.
