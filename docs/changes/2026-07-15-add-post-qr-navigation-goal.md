# 2026-07-15 Add Post-QR Navigation Goal

## Purpose

After the QR sequence succeeds, continue the task with a navigation goal relative to the vehicle's actual map pose.

## Changed files

- `ucar_ws/src/yolo2025/scripts/2026.py`
  - After the heading sequence completes with at least three distinct QR results, reads `map -> base_link` and sends a move_base/CymPlanner goal at `(current_x, current_y - 1.2)` with `yaw=pi`.
  - Adds completion/failure logs for the post-QR goal.
  - Skips the new goal if QR recognition is incomplete, scanning ends abnormally, or the current map pose cannot be read.
- `ucar_ws/src/yolo2025/launch/2026.launch`
  - Adds `post_qr_goal_enabled`, `post_qr_goal_y_offset=-1.2`, and `post_qr_goal_yaw=3.141593` parameters.
- `docs/operations.md`
  - Documents the post-QR movement and its safety conditions.

## Verification

- Local Python syntax validation passed.
- Local launch XML validation passed.

## Scope and limitation

- This change is local only and has not been uploaded to the vehicle or run on the vehicle.
- The downstream target depends on the live `map -> base_link` transform, rather than assuming the nominal QR waypoint coordinate.
