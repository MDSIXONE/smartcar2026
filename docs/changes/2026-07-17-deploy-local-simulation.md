# 2026-07-17 Deploy local Simulation in WSL Ubuntu 20.04

## Purpose

Deploy the `simulation` ROS Noetic/Gazebo Classic workspace to the local WSL
Ubuntu 20.04 runtime and verify the non-task preparation launch.

## Deployment

- Installed the missing local build and runtime dependencies: C++ compiler,
  Gazebo ROS integration/control packages, navigation stack, map server,
  controller packages, OpenCV, and `dos2unix`.
- Synced `simulation` from the Windows checkout to
  `/root/smartcar2026-simulation`, excluding generated build outputs, because
  CMake blocked on the Windows-mounted 9P filesystem.
- Normalized only the deployed Python scripts to Unix line endings and restored
  executable permissions. The Windows source checkout was not modified.
- Built all five workspace packages with `catkin_make -j2`.
- Started the only ROS Master at `http://192.168.8.197:11311`. Since that
  address was not assigned on the current disconnected host, it was temporarily
  added to the WSL loopback interface for local-only simulation.

## Verification

- `rosparam list` confirmed that the Master is live on port 11311.
- The headless `task3_prepare.launch` registered Gazebo, map server,
  move_base, robot state publisher, cube spawning, grasping, arm setup, and
  controller-spawner nodes.
- `/clock`, `/scan`, `/imu`, `/odom`, `/map`, camera/depth, TF, and arm action
  topics are advertised.
- `controller_manager/list_controllers` reports a running joint-state
  controller and initialized arm/gripper controllers. The
  `position_controllers/JointTrajectoryController` type is available.
- The final launch log contains no Python CRLF, syntax, RLException, or missing
  controller errors.

## Limitations

- The loopback address makes the fixed Master URI usable locally only. It does
  not make WSL reachable by the vehicle; reconnect the controller computer to
  the 192.168.8.0/24 network and use its real assigned address before vehicle
  integration.
- Gazebo Classic is deprecated upstream, but this workspace targets ROS Noetic
  and Gazebo 11.
