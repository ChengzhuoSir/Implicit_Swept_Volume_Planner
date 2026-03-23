cmake_minimum_required(VERSION 3.0.2)
project(isweep_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_conversions
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED)

file(GLOB ISWEEP_PLANNER_IGL_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.a")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions roscpp roslib sensor_msgs std_msgs tf2 tf2_ros visualization_msgs
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  include/isweep_planner/third_party
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})


# ==============================================================================
# 🚀 Hardware-Specific Optimizations (Targeted)
# ==============================================================================
# We define architecture-specific flags but only apply them to our own target
# (isweep_planner_core) to avoid corrupting the pre-compiled third-party static libs.

set(ISWEEP_HARDWARE_FLAGS "")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
  message(STATUS "iSweep: Detected ARM64 architecture. Enabling NEON acceleration.")
  set(ISWEEP_HARDWARE_FLAGS "-O3" "-mcpu=native" "-mtune=native")
  add_definitions(-DEIGEN_NEON)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "amd64")
  message(STATUS "iSweep: Detected x86_64 architecture. Enabling AVX/SSE acceleration safely.")
  set(ISWEEP_HARDWARE_FLAGS "-O3" "-march=native" "-mtune=native")
  add_definitions(-DEIGEN_MAX_ALIGN_BYTES=0)
endif()

add_library(isweep_planner_lmbm SHARED IMPORTED)
set_target_properties(isweep_planner_lmbm PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/lmbm.so
)

set(ISWEEP_PLANNER_SOURCES
  src/env/grid_map.cpp
  src/env/polygon_sdf.cpp
  src/env/body_frame_sdf.cpp
  src/env/footprint_model.cpp
  src/env/collision_checker.cpp
  src/ros_interface/occupancy_grid_bridge.cpp
  src/ros_interface/ros_visualizer.cpp
  src/global_search/topology_planner.cpp
  src/optimization/midend/se2_sequence_generator.cpp
  src/global_search/hybrid_astar.cpp
  src/global_search/swept_astar.cpp
  src/optimization/evaluator/svsdf_evaluator.cpp
  src/optimization/backend/trajectory_optimizer.cpp
  src/optimization/backend/se2_svsdf_solver.cpp
  src/env/implicit_svsdf_map/grid_map_3d.cpp
  src/env/implicit_svsdf_map/pcs_map_manager.cpp
  src/env/implicit_svsdf_map/mesh_shape_model.cpp
  src/env/implicit_svsdf_map/swept_volume_calculator.cpp
  src/optimization/svsdf_backend/svsdf_midend.cpp
  src/optimization/svsdf_backend/svsdf_backend_optimizer.cpp
  src/optimization/svsdf_backend/debug_publisher.cpp
  src/core/trajectory_utils.cpp
  src/core/math_utils.cpp
  src/framework/svsdf_runtime.cpp
  src/framework/svsdf_runtime_plan.cpp
  src/framework/svsdf_runtime_evaluate.cpp
  src/framework/recovery_manager_tail.cpp
  src/framework/recovery_manager_bottleneck.cpp
)

add_library(isweep_planner_core STATIC ${ISWEEP_PLANNER_SOURCES})
# Apply hardware acceleration ONLY to our source files
target_compile_options(isweep_planner_core PRIVATE ${ISWEEP_HARDWARE_FLAGS})

target_link_libraries(isweep_planner_core
  isweep_planner_lmbm
  OpenMP::OpenMP_CXX
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${ISWEEP_PLANNER_IGL_LIBS}
)

add_executable(isweep_planner_node src/ros_interface/isweep_planner_node.cpp)
target_link_libraries(isweep_planner_node
  isweep_planner_core
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

