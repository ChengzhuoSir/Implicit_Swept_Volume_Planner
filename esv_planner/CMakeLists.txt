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

set(ESV_THIRD_PARTY_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../third_party")
set(ESV_EIGEN_OVERRIDE_INCLUDE_DIR "${ESV_THIRD_PARTY_DIR}/eigen3")
set(ESV_LIBIGL_INCLUDE_DIR "${ESV_THIRD_PARTY_DIR}/libigl/include")
set(ESV_LBFGSPP_INCLUDE_DIR "${ESV_THIRD_PARTY_DIR}/LBFGSpp/include")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs visualization_msgs std_msgs tf2 tf2_ros
  DEPENDS EIGEN3
)

include_directories(
  include
  ${ESV_EIGEN_OVERRIDE_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${ESV_LIBIGL_INCLUDE_DIR}
  ${ESV_LBFGSPP_INCLUDE_DIR}
)

# --- Source groups (simplified after evaluator flattening) ---

set(ESV_MAP_SOURCES
  src/grid_map.cpp
  src/body_frame_sdf.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
)

set(ESV_SVSDF_SOURCES
  ${ESV_MAP_SOURCES}
  src/svsdf_evaluator.cpp
)

set(ESV_TOPOLOGY_SOURCES
  ${ESV_SVSDF_SOURCES}
  src/topology_planner.cpp
)

set(ESV_SEQUENCE_SOURCES
  ${ESV_TOPOLOGY_SOURCES}
  src/se2_sequence_generator.cpp
)

set(ESV_OPTIMIZER_SOURCES
  ${ESV_SEQUENCE_SOURCES}
  src/minco_parameterization.cpp
  src/trajectory_optimizer.cpp
)

# --- Main node ---

add_executable(esv_planner_node
  src/esv_planner_node.cpp
  ${ESV_OPTIMIZER_SOURCES}
)
target_link_libraries(esv_planner_node ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

catkin_install_python(
  PROGRAMS
    test/trajectory_generation_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/trajectory_generation.test)
endif()
