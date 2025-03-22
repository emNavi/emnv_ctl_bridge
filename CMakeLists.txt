cmake_minimum_required(VERSION 3.0.2)
project(ctrl_bridge)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  sensor_msgs
  # map_operation_pkg
  # path_searching_pkg
  # poly_curves_pkg
)
find_package(Eigen3 REQUIRED)

catkin_package(
  # CATKIN_DEPENDS 
  # CATKIN_DEPENDS map_operation_pkg
  # CATKIN_DEPENDS path_searching_pkg
  # CATKIN_DEPENDS poly_curves_pkg
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(ctrl_bridge src/ctrl_bridge_node.cpp src/mavros_utils.cpp)
target_link_libraries(ctrl_bridge
  ${catkin_LIBRARIES}
)
add_executable(ekf_test src/ekf_hover_test.cpp)
target_link_libraries(ekf_test
  ${catkin_LIBRARIES}
)
