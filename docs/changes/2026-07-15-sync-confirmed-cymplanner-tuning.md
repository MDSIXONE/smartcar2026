# 2026-07-15 Sync Confirmed CymPlanner Tuning Locally

## Purpose

Synchronize the local CymPlanner configuration with the vehicle tuning values confirmed by the operator.

## Changed files

- `ucar_ws/src/cym_planner/config/ucar_cym_planner_params.yaml`
  - Sets `max_vel_x` to `0.5 m/s`.
  - Sets `max_vel_y` to `0.1 m/s`.
  - Sets `linear_x_gain` to `1.5`.
  - Sets `linear_x_kd` to `0.5`.
  - Sets `angular_gain` (angular P) to `2.5`.
  - Sets `angular_kd` (angular D) to `0.4`.
- `docs/operations.md`
  - Updates the recorded ordinary navigation tuning values.

## Verification

- Compared the local configuration and the vehicle-side YAML before editing; both files still contained older values.
- Local YAML validation confirmed all six supplied tuning values.
- Vehicle-side launch parsing listed all expected nodes.
- First runtime attempt: wheel odometry repeatedly published `TF_NAN_INPUT` before a goal was sent; the launch was stopped.
- After an eight-second wait and a clean restart, all six tuning values loaded and `/odom` became finite. The default goal was rejected with action status `4` because CymPlanner reported a blocked path segment; QR and post-QR stages did not start. At that time the live global pose was `map (1.270, 2.710, 94.3 deg)`, different from the configured initial pose `(-0.25, 2.75, 0 deg)`.
- A requested third attempt again produced continuous `wheelodom` NaN values, so `move_base` did not expose its action server within 15 seconds. It was stopped before any goal was sent.

## Limitation

- The configuration is deployed and was loaded during the successful initialization portion of the second runtime attempt.
- A successful route test requires valid `odom -> base_link` TF, a matching `map -> base_link` start pose, and a clear local path to the first target.
