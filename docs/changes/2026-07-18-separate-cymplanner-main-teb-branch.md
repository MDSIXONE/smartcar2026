# 2026-07-18 Separate CymPlanner main and TEB trial branches

## Purpose

Keep the tested CymPlanner navigation on `main` and isolate the experimental
TEB navigation on a separate GitHub branch after the TEB trial prevented the
normal RViz workflow from loading correctly.

## Branches

- `main` is restored to the CymPlanner content from baseline `aa58c5b`.
- `teb-trial` preserves the complete TEB trial through commit `727da8f`.
- The main branch restoration uses normal revert commits, so no force push or
  destructive remote history rewrite was used.

## Vehicle restoration

- Restored `yolo2025/launch/2026.launch` from `main` to the vehicle.
- Restarted with `startup_goal_enabled:=false` and the WSL Master
  `http://192.168.8.197:11311`.
- Live `/move_base/base_local_planner` is
  `cym_planner/CymPlanner`; `/move_base`, `/navigation_2026`, and RViz are
  present.
- `/odom_raw`, `odom -> base_link`, and no-goal zero-motion checks passed.

## Known limitation

The TEB files remain available only on `teb-trial`; do not copy that branch's
`2026.launch` or TEB move_base launch into a stable CymPlanner deployment.
