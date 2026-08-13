# 2026-07-18 Fix TEB footprint model and obstacle clearance

## Purpose

Prevent the real vehicle from becoming infeasible at the first intersection
after switching the 2026 task to TEB.

## Cause

The TEB YAML used `footprint_model/types`, but TEB expects
`footprint_model/type`. The plugin therefore fell back to a point robot while
the costmap continued using the real rectangular footprint. The launch log
reported both the point-model fallback and an infeasible-radius warning.

The active TEB hard obstacle clearance was also `0.10 m`, which was more
conservative than required for this field.

## Changed files

- `ucar_ws/src/ucar_nav/config/omni_test20250620/teb_local_planner_params.yaml`
- `docs/operations.md`

## Change

- Correct `footprint_model/types` to `footprint_model/type` and keep the
  `0.342 m x 0.256 m` polygon used by both costmaps.
- Set `min_obstacle_dist` from `0.10 m` to `0.03 m` as requested.
- Remove unsupported precomputed radius fields; TEB derives them from the
  polygon vertices.

## Verification

- Before the fix, the static configuration probe failed with
  `min_obstacle_dist=0.1` and a missing `footprint_model/type`.
- The post-change static probe passed for the polygon vertices and `0.03 m`
  clearance.
- The file was deployed to the vehicle with matching SHA-256. After a no-goal
  restart, the live parameters returned `polygon` and `0.03`; neither the
  point-model fallback nor the infeasible-radius warning appeared.
- `/odom_raw` was finite and both `odom -> base_link` and `map -> base_link`
  transforms were available. No `/cmd_vel` message was emitted during the
  no-goal verification.

## Known limitations

The occasional scan drop caused by TF timestamps slightly ahead of the latest
transform is separate from this footprint/clearance correction. If it remains
frequent after this change, diagnose timing and transform tolerance separately.
