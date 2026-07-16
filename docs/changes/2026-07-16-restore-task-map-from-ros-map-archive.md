# Restore task map from ros_map archive

## Purpose

Restore the prior 5 m × 6 m task map selected by the user after a temporary
manually-scanned map had replaced it.

## Changed files

- `ucar_ws/src/ucar_nav/maps/iflysse_2026_direct.pgm`
  - replaced directly with `ros_map/iflysse_2026_direct.pgm` from
    `ros_map.zip`.
- `ucar_ws/src/ucar_nav/maps/iflysse_2026_direct.yaml`
  - restored from the paired archive entry.
- `docs/operations.md`
  - records the deploy and no-motion restart commands.

## Verification

- Restored PGM SHA-256:
  `2190e6e53d82bbfb8d4e040b5dab4c1b30f72324ba81a2f7795cb2d9eb548f5c`.
- Restored YAML references the paired PGM and specifies 0.01 m resolution with
  origin `[-2.5, -3.0, 0.0]`.
- Both map files were checksum-verified after upload to the vehicle. After one
  clean no-motion restart to clear a transient wheel-odometry NaN, map_server
  published the restored 500 × 600 map and `map -> base_link` recovered at
  approximately `(-0.180, 2.750, 0 degrees)`.

## Known limitation

`map_server` reads the map only at startup. The vehicle navigation launch must
be restarted after deployment before RViz can display the restored map.
