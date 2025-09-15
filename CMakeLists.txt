cmake_minimum_required(VERSION 3.0.2)
project(emnv_ctl_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  nav_msgs
)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_message_files(
  FILES
  PvayCommand.msg
)

generate_messages(
    DEPENDENCIES
    geometry_msgs
    nav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ctrl_bridge src/ctrl_bridge_node.cpp src/mavros_utils.cpp)
add_dependencies(ctrl_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ctrl_bridge
  ${catkin_LIBRARIES}
  yaml-cpp

)
add_executable(ekf_test src/ekf_hover_test.cpp)
add_dependencies(ekf_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf_test
  ${catkin_LIBRARIES}
)

add_executable(traj_test src/traj_test.cpp src/polynomial_traj.cpp)
add_dependencies(traj_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_test
  ${catkin_LIBRARIES}
)

set(SETUP_BASH_FILE "${CMAKE_BINARY_DIR}/../devel/setup.bash")
set(SCRIPT_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Tools/help_func.sh")


# 将脚本路径添加到 setup.bash 文件
add_custom_command(
  TARGET ctrl_bridge POST_BUILD
  COMMAND echo "source ${SCRIPT_PATH}" >> ${SETUP_BASH_FILE}
)