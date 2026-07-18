# 2026-07-18 Fix CymPlanner final-yaw pi-boundary oscillation

## Purpose

Prevent the real vehicle from alternating clockwise and counter-clockwise at
the goal when its heading is almost exactly opposite the requested final
heading.

## Root cause

The goal position had already passed the 0.05 m terminal-position check, but
`tf::getYaw()` alternated between values near `+pi` and `-pi` as localisation
noise crossed the angle branch cut.  With `final_yaw_gain: 2.0` and
`final_yaw_max_vel: 1.0`, this changed the command directly between
`+1.0 rad/s` and `-1.0 rad/s`.  `goal_reached_` therefore never became true,
so `move_base` correctly remained in its controller state and kept calling the
local planner.

## Changed files

- `ucar_ws/src/cym_planner/include/cym_planner/final_yaw_control.h`
- `ucar_ws/src/cym_planner/include/cym_planner.h`
- `ucar_ws/src/cym_planner/src/cym_planner.cpp`
- `ucar_ws/src/cym_planner/test/final_yaw_control_test.cpp`
- `ucar_ws/src/cym_planner/CMakeLists.txt`
- `ucar_ws/src/cym_planner/package.xml`
- `docs/operations.md`

## Change

- Unwrap consecutive final-yaw errors so the representation nearest the
  previous sample is used across the `+pi/-pi` boundary.
- Preserve terminal orientation state when the 3 Hz global replan has the same
  final position, orientation, and frame.
- Continue resetting the normal path target index for every new global plan,
  so path-following behavior before the terminal stage is unchanged.
- Reset the yaw tracker for a genuinely new goal and when terminal alignment
  begins.

## Verification

- The regression test failed against the old raw-yaw behavior in both rotation
  directions.
- Both branch-cut regression cases pass after the fix.
- An isolated Noetic catkin workspace builds `libcym_planner.so` and reports
  `4 tests, 0 errors, 0 failures, 0 skipped`.
- A direct C++11 syntax check of `cym_planner.cpp` passes.

## Known limitations

- The captured real-car recording cannot be replayed through the ROS control
  loop because it contains video rather than `/tf`, `/cmd_vel`, and plan topic
  data.  Real-car validation is still required after deployment.
- The fix does not suppress or compensate for invalid/NaN odometry.  Existing
  safety rules still require zero velocity and navigation/odometry restart
  before movement if NaN odometry or `TF_NAN_INPUT` appears.

## Deployment status

Implemented and verified locally only.  It has not been copied to or built on
the vehicle in this change.
