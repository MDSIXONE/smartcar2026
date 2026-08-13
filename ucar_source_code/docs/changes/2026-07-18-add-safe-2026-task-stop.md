# 2026-07-18 Add safe 2026 task stop command

## Purpose

Provide a deterministic way to stop the real-vehicle 2026 task when its
original launch terminal was closed without stopping the task.

## Changed files

- `ucar_ws/src/yolo2025/scripts/stop_2026_task.sh`
- `docs/quickstart.md`
- `docs/operations.md`

## Behavior

The script explicitly reconnects to the WSL Master, publishes a zero
`/cmd_vel`, and stops only the matching `yolo2025 2026.launch` process and its
direct child processes.  It pauses the launch supervisor before terminating
children so `move_base` cannot respawn.  It never starts or stops a ROS Master.

## Verification

- Shell syntax is checked with `bash -n`.
- The process matching is limited to the 2026 launch command.

## Limitation

The script does not remove stale ROS Master registrations.  When needed,
operators must inspect the node list and run the documented interactive
`rosnode cleanup` command only for unreachable old 2026 nodes.
