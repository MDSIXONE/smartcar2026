# Calibrate the lidar static yaw to zero

## Purpose

Remove the approximately four-degree clockwise offset between the laser scan
and the static map that was present immediately after startup while the car was
stationary.

## Changed files

- `ucar_ws/src/ucar_controller/launch/ucar_bringup.launch`
  - changes the `base_link -> laser_frame` static-transform yaw from `-0.07`
    rad to `0.0` rad; the measured lidar bracket is treated as parallel to the
    chassis for this calibration.
- `docs/operations.md`
  - documents the calibrated extrinsic, deployment/restart commands, and the
    stationary verification rule.

## Evidence and scope

- The observed scan/map error was a constant clockwise offset from startup,
  rather than an error accumulating only while turning.
- The prior static transform supplied exactly `-0.07 rad` of yaw, equivalent
  to approximately `-4.0°`, which matches the reported offset.
- No scan-matcher sign, odometry/IMU convention, map, costmap, or planner/PD
  setting was changed.

## Deployment verification

- Launch XML parses successfully and its static-transform arguments were
  verified locally as yaw `0.0`.
- Uploaded `ucar_bringup.launch` to the vehicle without creating a vehicle-side
  backup.
- The deployed file SHA-256 matches the local file:
  `4df7d0a0ce5c9e5b89f6d4b4904b6e3be20e26ecd704447adcdb70e42611a968`.
- The vehicle launch process was deliberately not started or restarted; the
  user will start it and thereby replace the old static-TF publisher.

## Verification plan

1. Validate the launch XML locally and confirm the deployed file checksum.
2. Restart the complete no-goal navigation/bringup chain so the old static TF
   publisher exits.
3. With the vehicle stationary, verify `base_link -> laser_frame` yaw is zero
   and confirm `/scan` overlays static walls in RViz.
4. Only after `/odom_raw` and both required TFs are finite may a rotation test
   assess the separate odometry-prediction yaw issue.

## Known limitation

If the physical lidar is not actually parallel to the chassis, a residual
constant error must be corrected by measuring and setting the true yaw rather
than by changing map or localization conventions.
