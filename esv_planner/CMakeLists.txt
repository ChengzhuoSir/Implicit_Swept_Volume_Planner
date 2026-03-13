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
find_package(OpenMP REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs visualization_msgs std_msgs tf2 tf2_ros
  DEPENDS EIGEN3
)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(esv_planner_lmbm SHARED IMPORTED)
set_target_properties(esv_planner_lmbm PROPERTIES
  IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/lmbm.so
)

set(ESV_PLANNER_SOURCES
  src/grid_map.cpp
  src/polygon_sdf.cpp
  src/body_frame_sdf.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/svsdf_evaluator.cpp
  src/minco_parameterization.cpp
  src/grid_obstacle_source.cpp
  src/local_obstacle_cache.cpp
  src/swept_astar.cpp
  src/path_sparsifier.cpp
  src/lmbm_solver.cpp
  src/true_svsdf_penalty.cpp
  src/paper_backend.cpp
  src/paper_planner.cpp
)

add_library(esv_planner_core ${ESV_PLANNER_SOURCES})
target_link_libraries(esv_planner_core
  esv_planner_lmbm
  OpenMP::OpenMP_CXX
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

# --- Main node ---

add_executable(esv_planner_node
  src/esv_planner_node.cpp
)
target_link_libraries(esv_planner_node esv_planner_core ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES})

catkin_install_python(
  PROGRAMS
    test/paper_pipeline_smoke.py
    test/trajectory_generation_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(esv_planner_geometry_test test/planner_geometry_test.cpp)
  if(TARGET esv_planner_geometry_test)
    target_link_libraries(esv_planner_geometry_test
      esv_planner_core
      gtest_main
      ${catkin_LIBRARIES}
      ${EIGEN3_LIBRARIES}
    )
  endif()
  add_rostest(test/trajectory_generation.test)
  add_rostest(test/paper_pipeline_smoke.test)
endif()
