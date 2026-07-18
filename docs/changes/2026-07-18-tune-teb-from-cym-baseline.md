# 2026-07-18 Tune TEB from the CymPlanner baseline

## Purpose

Run the complete real-vehicle 2026 task with TEB while retaining the speed,
goal precision, vehicle size, and narrow-field clearance already proven with
CymPlanner.

## Changed files

- `ucar_ws/src/ucar_nav/config/omni_test20250620/teb_local_planner_params.yaml`
- `docs/operations.md`

## Change

- Limit forward and backward speed to `0.25 m/s`, the Cym production-route
  limit, and lateral speed to the Cym `0.10 m/s` limit.
- Match Cym's `1.0 rad/s` angular limit and use `0.5 rad/s^2` angular
  acceleration so the six-second QR heading goals remain achievable.
- Match Cym's `0.05 m` position-entry threshold and `0.10 rad` final-yaw
  tolerance.
- Retain the real `0.342 m x 0.256 m` polygon and `0.03 m` hard clearance.
- Increase global-plan following density to `0.10 m`, increase viapoint weight
  to `10`, and disable homotopy-class alternatives for a deterministic first
  grid-route run.
- Reduce `penalty_epsilon` to `0.05`, so it is not larger than the configured
  `0.10 m/s` lateral limit.

## Verification

- The pre-change static comparison failed on nine mismatched TEB values.
- The post-change static comparison must pass before deployment.
- Deploy and start with `startup_goal_enabled:=false`; confirm the live local
  planner is TEB and all parameters above are active before motion.
- Only run after finite `/odom_raw` and valid `odom -> base_link` and
  `map -> base_link` transforms are confirmed.

## Known limitations

This is a conservative first-run baseline, not a final TEB optimization. The
intermittent scan/TF future-extrapolation warnings are a separate timing issue.
