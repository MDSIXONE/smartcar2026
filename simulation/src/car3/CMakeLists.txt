cmake_minimum_required(VERSION 2.8.3)

project(car3)

find_package(catkin REQUIRED)

catkin_package()

find_package(roslaunch)

if(CATKIN_ENABLE_TESTING)
  find_package(PythonInterp REQUIRED)
  add_test(NAME cube_mesh_uri
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cube_mesh_uri.py)
  add_test(NAME sign_mesh_uri
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sign_mesh_uri.py)
  add_test(NAME scene_sign_compat
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_scene_sign_compat.py)
  add_test(NAME task3_visual_pipeline
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_task3_visual_pipeline.py)
  add_test(NAME grasp_attach_state
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_grasp_attach_state.py)
  add_test(NAME task3_pick_sequence
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_task3_pick_sequence.py)
  add_test(NAME task3_carry_sequence
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_task3_carry_sequence.py)
  add_test(NAME task3_retry_state_machine
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_task3_retry_state_machine.py)
endif()

catkin_install_python(PROGRAMS
  scripts/cmd_vel_arbiter.py
  scripts/gripper_mimic.py
  scripts/grasp_attach.py
  scripts/reset_car_start.py
  scripts/scene_sign_compat.py
  scripts/set_arm_initial_pose.py
  scripts/task3_pick_deliver.py
  scripts/spawn_cubes.py
  scripts/odom_tf_broadcaster.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch meshes models urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
