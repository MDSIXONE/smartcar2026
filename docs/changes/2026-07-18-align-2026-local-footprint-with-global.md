# 2026-07-18 Align 2026 local footprint with global costmap

## Purpose

Use the same physical footprint in the global costmap, local costmap, RViz
Marker, and RViz visual model so collision checks and visual feedback agree.

## Changed files

- `ucar_ws/src/ucar_nav/config/omni_test20250620/local_costmap_params.yaml`
- `ucar_ws/src/yolo2025/scripts/2026.py`
- `ucar_ws/src/yolo2025/launch/2026.launch`
- `ucar_ws/src/yolo2025/urdf/ucar_2026_visual.urdf`
- `docs/quickstart.md`
- `docs/operations.md`

## Footprint

All four consumers use
`[[0.171, -0.128], [0.171, 0.128], [-0.171, 0.128], [-0.171, -0.128]]`,
which is `0.342 m × 0.256 m`.

## Verification

- Parse the local-costmap YAML, launch XML, URDF XML, and RViz YAML.
- Assert that every consumer has the same footprint dimensions.

## Deployment

Synchronize the local-costmap YAML, `2026.py`, `2026.launch`, and visual URDF
to the vehicle.  Restart `2026.launch` with `startup_goal_enabled:=false` and
then restart RViz before any movement test.
