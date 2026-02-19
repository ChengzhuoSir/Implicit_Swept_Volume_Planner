cmake_minimum_required(VERSION 3.0.2)
project(esv_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs visualization_msgs std_msgs tf2 tf2_ros
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(esv_planner_node
  src/esv_planner_node.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(esv_planner_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
