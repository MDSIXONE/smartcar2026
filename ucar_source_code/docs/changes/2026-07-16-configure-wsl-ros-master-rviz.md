# 2026-07-16 Configure WSL ROS Master and RViz

## Purpose

Use the local WSL Ubuntu 20.04 instance as the LAN-accessible ROS Master and
provide RViz locally, so the vehicle can publish its ROS graph to the
controller computer.

## Configuration

- Installed ROS Noetic ros-core and rviz in WSL.
- Configured WSL's shell environment for ROS_MASTER_URI and ROS_IP at
  192.168.8.197.
- Added ~/start_ros_master.sh for a foreground Master process in an interactive
  WSL terminal. This WSL installation does not currently use systemd, so no
  system service is enabled.
- Added ~/start_rviz.sh. It uses a private root runtime directory, X11, and
  software OpenGL because the WSLg shared Wayland runtime belongs to UID 1000
  while this distribution's default user is root.
- Disabled the ROS 1 Noetic EOL dialog in the WSL shell and both launch
  scripts with DISABLE_ROS1_EOL_WARNINGS=1.

## Verification

- WSL is version 2 with mirrored networking and holds 192.168.8.197/24.
- ROS package installation completed successfully; roscore and rviz are
  installed.
- The persistent Windows-hosted WSL process starts roscore successfully and
  listens on 0.0.0.0:11311; local rosparam and RViz Master checks pass.
- RViz opens through WSLg. Its temporary launch remained running until the
  expected test timeout.
- The root-safe RViz launcher starts without the prior WSLg runtime-directory
  ownership warning and stays connected to the configured Master.
- The vehicle-to-WSL port test timed out because Windows Hyper-V firewall
  policy requires administrator-created inbound rules. The required scoped
  commands are documented in operations.md.
- After the firewall rules were applied, a ROS Melodic vehicle publisher
  delivered the temporary wsl_ros_link_test string to a ROS Noetic WSL
  subscriber. This verifies the Master and the dynamic TCPROS connection in
  the direction used by vehicle publishers and local RViz subscribers.
- After restarting WSL, the Master, and RViz with
  `DISABLE_ROS1_EOL_WARNINGS=1`, the ROS 1 EOL dialog no longer appeared;
  only the normal RViz main window remained.

## Limitation

- The vehicle must set its ROS_MASTER_URI to the WSL address before launch.
  It must not run a second Master concurrently.
- The current Master is hosted by a background WSL process. After a Windows
  reboot or WSL shutdown, run ~/start_ros_master.sh again before starting
  vehicle ROS nodes.
- The helper first probes the configured Master and exits successfully when it
  is already running, preventing a misleading second-roscore exception.
- The vehicle `.bashrc` intentionally defaults to its on-board Master at
  `192.168.8.231`. For WSL RViz operation, launch commands must run
  `unset ROS_HOSTNAME` and override `ROS_IP` plus `ROS_MASTER_URI` after all
  setup files have been sourced; otherwise the vehicle stays isolated on its
  on-board Master.
- A stale WSLg RDP/RAIL session produced a taskbar-only RViz window titled
  [WARN:COPY MODE]. Restarting WSL, then restarting the Master and RViz,
  removed that state; a Windows capture verified the full RViz main window,
  toolbar, panels, and render viewport are visible.
