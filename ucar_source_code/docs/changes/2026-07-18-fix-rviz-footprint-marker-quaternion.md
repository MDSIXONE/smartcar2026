# 2026-07-18 Fix RViz footprint Marker quaternion

## Purpose

Make the red physical-footprint and yellow safety-margin `LINE_STRIP` Markers
render in RViz instead of being rejected as an uninitialised quaternion.

## Changed files

- `ucar_ws/src/yolo2025/scripts/2026.py`
- `docs/operations.md`

## Change

Each `/navigation_2026/footprint` Marker now explicitly uses the identity
orientation (`pose.orientation.w = 1.0`) while remaining fixed in `base_link`.
The Marker remains visual-only and does not publish TF or affect planning,
costmaps, or motion.

## Verification

- Python syntax check passes.
- Static source check confirms the Marker has an identity quaternion before it
  is published.

## Deployment

Copy `2026.py` to the vehicle and restart `2026.launch` with
`startup_goal_enabled:=false`.  Restart RViz only if it has not automatically
redrawn the latched Marker after the task node restarts.
