# 2026-07-18 Fix 2026 scan startup race on the TEB trial branch

## Purpose

Keep the TEB run's scan forwarding and localization chain alive from the first
laser frame.

## Cause

`2026.py` created the `/scan_raw` subscriber before initializing fields read by
`scan_cb`. The first callback could therefore raise
`AttributeError: production_global_obstacles_frozen`, preventing the filtered
scan/global-obstacle path from being reliable during planner startup.

## Changed files

- `ucar_ws/src/yolo2025/scripts/2026.py`
- `docs/operations.md`

## Change

Initialize all scan-filter state and `production_global_obstacles_frozen` before
constructing the subscriber. No topic, frame, or planner parameter changes are
made.

## Verification

- The previous TEB no-goal launch reproduced the callback exception in its
  log.
- Run `python2 -m py_compile` before deployment.
- After a no-goal restart, the log must contain no `bad callback` or
  `production_global_obstacles_frozen` error, and `map -> base_link` must become
  available before enabling the automatic goal.

## Safety

The automatic goal remains disabled until the no-goal checks pass. Before any
motion, verify finite `/odom_raw` and valid `odom -> base_link` and
`map -> base_link` transforms.
