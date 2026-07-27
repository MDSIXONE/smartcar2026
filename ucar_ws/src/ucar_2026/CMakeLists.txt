cmake_minimum_required(VERSION 2.8.3)
project(ucar_2026)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/navigation_scan_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/stop_2026_task.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
