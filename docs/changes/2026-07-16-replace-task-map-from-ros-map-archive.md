# Replace task map from ros_map archive

## Purpose

Replace the active task map with the map supplied in `ros_map.zip`, without
changing the IMU, lidar-localization, costmap, or planner configuration.

## Changed files

- `ucar_ws/src/ucar_nav/maps/iflysse_2026_direct.pgm`
  - replaced with `ros_map/iflysse_2026_direct.pgm` from `ros_map.zip`.
- `ucar_ws/src/ucar_nav/maps/iflysse_2026_direct.yaml`
  - replaced with the paired archive configuration.
- `ucar_ws/src/ucar_nav/maps/iflysse_2026_direct_metadata.json`
  - updated from the archive; line endings are normalized to the repository
    convention only.
- `ucar_ws/src/ucar_nav/maps/iflysse_2026_direct_preview.png`
  - updated from the archive preview.
- `docs/operations.md`
  - records the archive fingerprints and deployment scope.

## Validation

- Archive SHA-256:
  `f595400e2244fdc12e4f421475a03ed4964dc69eca81742244a538c7817c82a9`.
- The active PGM SHA-256 is
  `ea078e4af7648e0e39a340b8d6b8625e2a24ba85f4f022398d81a95cd7b58750`;
  the paired YAML SHA-256 is
  `1cdad0e7008f827ee37f246722dc79e9a2336a39faaff2a68f7a94458db627eb`.
- The PGM header is `P5`, `500 x 600`, maximum value `255`; the YAML specifies
  resolution `0.01 m` and origin `[-2.5, -3.0, 0.0]`.
- The map uses a 5 m x 6 m footprint. Metadata and preview dimensions agree
  with the PGM.
- The uploaded vehicle PGM and YAML SHA-256 values match the validated local
  files exactly.

## Deployment and limitation

- Upload only the active PGM and YAML to
  `~/ucar_ws/src/ucar_nav/maps/` on the vehicle; no vehicle-side backup is
  created.
- `map_server` reads the map only at launch. The navigation process is not
  restarted as part of this replacement; the next user-initiated launch will
  load the new map.
