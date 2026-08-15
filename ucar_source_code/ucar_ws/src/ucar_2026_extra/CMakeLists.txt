cmake_minimum_required(VERSION 2.8.3)
project(ucar_2026_extra)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/cmd_vel_owner.py
  scripts/navigation_scan_relay.py
  scripts/production_camera_ocr.py
  scripts/production_task_2026.py
  scripts/production_task_geometry.py
  scripts/production_task_perception.py
  scripts/yolo_dataset_capture.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/run_melodic_python2.sh
  scripts/start_2026.sh
  scripts/stop_2026_task.sh
  scripts/handoff_lane.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch urdf rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_production_task_geometry.py)
  catkin_add_nosetests(test/test_production_task_perception.py)
  add_test(
    NAME production_camera_ocr_py3
    COMMAND /usr/bin/python3
            ${CMAKE_CURRENT_SOURCE_DIR}/test/test_production_camera_ocr.py
  )
endif()
