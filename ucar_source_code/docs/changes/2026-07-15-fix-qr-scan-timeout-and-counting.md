# 2026-07-15 Fix QR Scan Timeout and Counting

## Purpose

Prevent a QR heading goal from rotating until the global scan timeout, and ensure valid camera detections are counted even when the configured stop duration is only 0.5 seconds.

## Evidence

- Vehicle log: the scanner detected `a`, `d`, and `i`, but `navigation_2026` recorded `codes=0/3` because detections fell outside the 0.5-second hold callbacks.
- Vehicle log: the final `i` heading action never returned success; it rotated from 21:53:04 until the 40-second global scan timeout cancelled it.
- The holonomic stage had not started, so `linear.y` was not involved in this failure.

## Changed files

- `ucar_ws/src/yolo2025/scripts/2026.py`
  - Counts unique QR results throughout the active QR phase.
  - Uses the live `map -> base_link` position for each same-place heading goal, avoiding a stale fixed scan-point position.
  - Adds a per-heading action timeout; cancellation enters the normal short scan hold rather than failing the task.
- `ucar_ws/src/yolo2025/launch/2026.launch`
  - Adds `qr_heading_goal_timeout=6.0` seconds and sets the evidence-based QR hold duration to `3.5` seconds.
- `docs/operations.md`
  - Documents QR result accounting and the per-heading rotation timeout.

## Verification

- Local Python syntax validation passed.
- Local launch XML validation passed.
- Deployed `2026.py` and `2026.launch` to the vehicle.
- Vehicle-side Python 2 compilation and `roslaunch --nodes yolo2025 2026.launch` validation passed.
- A subsequent vehicle run proved that `0.5` seconds is insufficient: the navigation task received only `a`, while scanner logs produced `i` and `d` after the task had already ended. The configured hold is therefore `3.5` seconds for the next verification.
- Final vehicle verification passed: `d`, `a`, and `i` were counted as `3/3`; the post-QR relative goal reached `(-1.760, 1.030, yaw=pi)`, and the fixed holonomic goal `(-2.134, -0.095, yaw=pi/2)` reached successfully before normal mode was restored.

## Scope and limitation

- This change does not alter `max_vel_x=0.5`, `angular_p=2.5`, or `angular_d=0.4`; those runtime values are retained as requested.
- The task was stopped after the successful vehicle verification; no navigation process remains running.
