# Replace task map from updated ros_map archive

## Purpose

Replace the active task map with the newer user-supplied `ros_map.zip` map,
without changing lidar localization, IMU, costmaps, or planner configuration.

## Changed files

- `ucar_ws/src/ucar_nav/maps/iflysse_2026_direct.pgm`
  - replaced with the newer archive PGM.
- `ucar_ws/src/ucar_nav/maps/iflysse_2026_direct.yaml`
  - replaced with its paired archive configuration.
- `ucar_ws/src/ucar_nav/maps/iflysse_2026_direct_metadata.json`
  - updated from the archive; line endings were normalized to the repository
    convention only.
- `ucar_ws/src/ucar_nav/maps/iflysse_2026_direct_preview.png`
  - updated from the archive preview.
- `docs/operations.md`
  - updates the current archive fingerprints.

## Validation

- Archive SHA-256:
  `333760de3ca64de36906833f7cea895a52ae9979694dfeb3b45c6a4e0ec1d01a`.
- Active PGM SHA-256:
  `2308ab7d197720ec1e50701727ed5a72a1d9ba551c4bf5371126257d867f9f9b`.
- Paired YAML SHA-256:
  `1cdad0e7008f827ee37f246722dc79e9a2336a39faaff2a68f7a94458db627eb`.
- PGM header is `P5`, `500 x 600`, maximum value `255`; the paired YAML has
  `0.01 m` resolution and origin `[-2.5, -3.0, 0.0]`.
- The metadata reports 11,316 occupied cells, so this is a content update from
  the prior map while retaining the same 5 m x 6 m coordinate convention.
- Vehicle-side PGM and YAML SHA-256 values were checked after upload and match
  the validated local files exactly.

## Deployment and limitation

- Upload only the PGM and YAML to
  `~/ucar_ws/src/ucar_nav/maps/` on the vehicle; do not create a vehicle-side
  backup.
- The navigation process is not restarted as part of this update. The next
  user-initiated launch is required for `map_server` to load the replacement.
