# 2026-07-27 Port current simulation CymPlanner to the real-vehicle workspace

## Purpose

Apply the current simulation `main_legacy` path follower and projected-footprint lidar guard to
the real-vehicle CymPlanner without copying Gazebo-specific frames, topics, or unsafe speed
values.

## Changed files

- `ucar_ws/src/cym_planner/src/cym_planner.cpp` and `include/cym_planner.h`: import the current
  simulation planner, including navigation-mode selection, path projection, blocked-state
  handling, command-rate helpers, and diagnostics. Real-vehicle topic adaptations use
  `/scan_filtered`, `/ucar/carry_mode`, and `/ucar/navigation_mode`.
- `include/cym_planner/velocity_profile.h` and `test/test_velocity_profile.cpp`: import the
  simulation command-rate helper and its regression suite.
- `config/ucar_cym_planner_params.yaml`: default to `laser_avoidance`, retain the measured
  real-vehicle footprint chain, and cap commands at `0.5 m/s` and `1.0 rad/s`.
- `CMakeLists.txt`: retain the existing final-yaw test and add the velocity-profile test.
- `README.md`, `config/README.md`, and `docs/operations.md`: document the new control path,
  deployment files, build/test commands, ROS Master, and pre-motion checks.

## Safety adaptations

- The vehicle keeps `base_link` and filtered `/scan_filtered`; simulation `/scan` is not used.
- The vehicle defaults to direct lidar projected-footprint guarding for the entire route.
- The simulation's `14 m/s` linear and `20.5 rad/s` angular limits are not copied. Existing
  vehicle limits remain `0.5 m/s` and `1.0 rad/s`, both in YAML and in C++ fallback defaults.
- No physical-vehicle files are changed and no motion command is sent by this migration.

## Verification

- YAML parsing confirms `laser_avoidance`, `/scan_filtered`, `0.5 m/s`, `1.0 rad/s`, and the
  `0.30 m` projected-path lookahead. Package and active move_base launch XML parse successfully.
- Static source checks confirm that unsafe simulation fallback limits are absent from the vehicle
  source and that both YAML and C++ defaults are fail-safe vehicle values.
- An isolated WSL Ubuntu 20.04 / ROS Noetic catkin workspace builds `libcym_planner.so` from the
  migrated source successfully.
- The retained final-yaw tests and imported velocity-profile tests pass: `14 tests, 0 errors,
  0 failures, 0 skipped` in `catkin_test_results`.
- The original real-vehicle planner, header, YAML, and CMake file are preserved under
  `back/2026-07-27-before-simulation-cymplanner-port/`.

## Known limitations

Physical motion behavior remains to be verified by the user. Before testing, confirm finite
`/odom_raw`, both required TF chains, a fresh `/scan_filtered`, and the unique WSL ROS Master at
`http://192.168.8.197:11311`.

The inherited `cym_planner_plugin.xml` contains a legacy non-UTF-8 description string. It is
unchanged by this port and the planner library builds successfully, but generic strict UTF-8 XML
parsers reject that description. Normalize it separately before any metadata-only tooling that
requires strict UTF-8.
