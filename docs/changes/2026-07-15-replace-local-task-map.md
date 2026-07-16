# 2026-07-15 Replace Local Task Map

## Purpose

Replace the local occupancy-grid image used by the 2026 navigation task with the updated map supplied in `ros_map.zip`.

## Changed file

- `ucar_ws/src/ucar_nav/maps/iflysse_2026_direct.pgm`
  - Replaced with `ros_map/iflysse_2026_direct.pgm` from the supplied archive.
  - SHA-256 after replacement: `2190e6e53d82bbfb8d4e040b5dab4c1b30f72324ba81a2f7795cb2d9eb548f5c`.

## Verification

- The PGM header is valid (`P5`, `500 x 600`, 8-bit pixels).
- The accompanying YAML in the archive is byte-identical to the existing
  `iflysse_2026_direct.yaml`; resolution and origin remain `0.01 m/pixel` and
  `[-2.5, -3.0, 0.0]`.
- `yolo2025/launch/2026.launch` already loads this YAML, so no launch change is needed.

## Scope and limitation

- This change is local only. The updated map has not been uploaded to the vehicle and no ROS task was started.
- The archive preview and metadata files were not copied because `map_server` uses only the PGM and YAML pair.
