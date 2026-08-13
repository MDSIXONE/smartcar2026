# 2026-07-18 Add 2026 RViz visual model

## Purpose

Make the real-vehicle RViz preset display a visible vehicle model instead of
only the TF arrow and footprint outlines.

## Changed files

- `ucar_ws/src/yolo2025/urdf/ucar_2026_visual.urdf`
- `ucar_ws/src/yolo2025/launch/2026.launch`
- `ucar_ws/src/yolo2025/rviz/navigation_2026.rviz`
- `docs/quickstart.md`
- `docs/operations.md`

## Design

The URDF has one existing `base_link` root and only visual geometry: a
footprint-matched `0.36 m × 0.24 m` chassis, front heading panel, lidar, and
four wheels.  It adds no joints, TF publishers, collision geometry, or control
parameters.  The launch loads it into `/robot_description`; the RViz preset
enables a `RobotModel` display for that parameter.

## Verification

- Parse the URDF and RViz YAML structure.
- Verify the launch references the URDF and the RViz preset enables the
  `RobotModel` display.

## Deployment

Synchronize the URDF and `2026.launch` to the vehicle, then restart the
vehicle's `2026.launch` with `startup_goal_enabled:=false` and restart RViz.
