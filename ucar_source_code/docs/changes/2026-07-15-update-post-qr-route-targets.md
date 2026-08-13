# 2026-07-15 Update Post-QR Route Targets

## Purpose

Replace all three post-QR targets with the operator-provided map-frame goals.

## Route

1. Normal mode: `(-1.737, 1.003, yaw=3.140)`.
2. Normal mode: `(-1.722, -0.269, yaw=-3.140)`.
3. Holonomic mode: `(-2.265, -0.001, yaw=-1.557)`.

All three translational stages remain capped at `0.1 m/s`.

## Changed files

- `ucar_ws/src/yolo2025/scripts/2026.py`
  - Replaces the relative second goal with a fixed map-frame goal and updates all defaults.
- `ucar_ws/src/yolo2025/launch/2026.launch`
  - Supplies the three new targets at runtime.
- `docs/operations.md`
  - Records the current route and mode sequence.

## Verification

- Local Python syntax and launch XML validation passed.
- Uploaded `2026.py` and `2026.launch` to the vehicle.
- Vehicle-side Python 2 compilation and `roslaunch --nodes yolo2025 2026.launch` validation passed.
- The navigation task remains stopped; the new target coordinates have not been run yet.
