# 2026-07-15 Decouple Post-QR Route Start

## Purpose

Ensure the first post-QR navigation goal is sent after the QR hold timer callback has returned.

## Changed files

- `ucar_ws/src/yolo2025/scripts/2026.py`
  - Schedules the post-QR route with a one-shot `0.1 s` ROS timer instead of starting it directly inside the QR hold timer callback.
  - Sends each post-QR goal before emitting its terminal status, avoiding a ROS logging call on the task-control path.
- `docs/operations.md`
  - Documents the handoff and the expected terminal marker.

## Diagnosis

- A vehicle run recognized `d`, `a`, and `i`, and wrote `task_max_vel=0.1`, but no `POST_QR_FIRST_GOAL` action goal was published.
- An external `rosparam set` of the same value completed successfully, so the parameter server was not the blocker.
- The timer decoupling allowed the task-speed log to complete, but the first goal still was not published; the remaining control-path operation before goal publication was the ROS progress log.

## Verification

- Local and vehicle-side Python syntax checks and launch-node parsing passed.
- A vehicle run reached the default target, recognized `d`, `a`, and `i`, then reached all three post-QR stages. The final move_base status was `3` (`Goal reached.`) at map pose approximately `(-2.070, -0.070, 85.0 deg)` for the then-configured third target.
- Progress status uses flushed standard output so `[POST_QR]` lines appear immediately in the launch terminal.

## Limitation

- The new timer adds `0.1 s` between QR completion and the first post-QR goal; it does not change the three route targets or their `0.1 m/s` translational cap.
