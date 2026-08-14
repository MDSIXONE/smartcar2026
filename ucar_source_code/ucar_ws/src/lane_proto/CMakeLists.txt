cmake_minimum_required(VERSION 2.8.3)
project(lane_proto)
find_package(catkin REQUIRED)
catkin_package()
install(PROGRAMS scripts/lane_follow.py scripts/trackseg.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_lane_runtime.py)
endif()
