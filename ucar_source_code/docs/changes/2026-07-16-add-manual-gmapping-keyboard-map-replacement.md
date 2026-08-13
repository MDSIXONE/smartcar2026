# 2026-07-16 Manual Gmapping Keyboard Map Replacement

## Purpose

Provide a safe way to rebuild the task map when the physical course no longer
matches the static navigation map, without running automatic navigation during
the mapping pass.

## Changed files

- `ucar_ws/src/yolo2025/launch/mapping.launch`
  - Starts only the normal UCar bringup and `slam_gmapping`.
  - Starts an unmodified `/scan_raw` to `/scan` relay because the vehicle's
    ydlidar launch publishes only `/scan_raw` while gmapping consumes `/scan`.
  - Does not start the navigation stack, static `map_server`, localization,
    camera, or automatic task node, avoiding competing `/map` or `/cmd_vel`
    publishers.
- `ucar_ws/src/yolo2025/scripts/mapping_keyboard.py`
  - Adds direct-terminal keyboard control with an automatic zero-velocity
    watchdog.
  - Allows live, bounded adjustment of the linear and angular mapping speeds.
  - `s` saves the current gmapping `/map` to a temporary `/tmp` path.
  - `t` requires that session save, then atomically replaces the two active
    `iflysse_2026_direct` map files and deletes the temporary files. It does
    not create a vehicle-side backup.
- `ucar_ws/src/yolo2025/scripts/mapping_scan_relay.py`
  - Relays only the raw laser scan required by gmapping; it does not filter or
    alter range data.
- `ucar_ws/src/yolo2025/CMakeLists.txt` and `package.xml`
  - Declare/install the keyboard script and its ROS runtime dependencies.
- `docs/operations.md`
  - Documents deploy, build, launch, controls, and safe handoff back to the
    normal navigation launch.

## Verification

- Local Python syntax, package XML, launch XML, and task-map YAML image-name
  rewrite checks passed.
- Vehicle-side `gmapping` and `map_server` executable availability was
  verified; both are installed in ROS Melodic.
- The changed files were uploaded to the vehicle. `catkin_make --pkg yolo2025`
  completed successfully.
- Vehicle-side Python 2 parsing and `roslaunch --nodes yolo2025 mapping.launch`
  passed; the launch resolves only bringup, laser, and `slam_gmapping` nodes.
- A non-interactive invocation exits before ROS-node initialization with status
  `2`, proving it cannot accidentally publish a motion command during launch
  validation.
- The live speed-adjustment bounds were unit-checked locally and with the
  vehicle's Python 2 interpreter; a positive adjustment changes each speed by
  its configured step and limits are enforced.

## Follow-up diagnosis

- The first live mapping attempt had a `slam_gmapping` publisher for `/map`,
  but no publisher for its `/scan` input. The UCar's ydlidar launch hardcodes
  its output as `/scan_raw`; the initial mapping launch omitted the relay.
- The regression check is that `roslaunch --nodes yolo2025 mapping.launch`
  includes `mapping_scan_relay`, and that `/scan` has that node as publisher
  before `s` is pressed.

## Limitations

- Gmapping relies on wheel/IMU odometry while the user manually drives. Drive
  slowly with overlapping laser views and close the loop before saving.
- The newly installed static map is loaded only after the mapping launch has
  stopped and `2026.launch` is started again.
