cmake_minimum_required(VERSION 2.8.3)
project(lane_proto)
find_package(catkin REQUIRED)
catkin_package()
# lane_follow 直接 import lane_common / yolo_client, 不装的话
# catkin_make install 之后会 ImportError(devel 空间跑看不出来)
install(PROGRAMS scripts/lane_follow.py scripts/trackseg.py
                scripts/lane_common.py scripts/yolo_client.py
                scripts/yolo_probe.py scripts/lidar_check.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_lane_runtime.py)
endif()
