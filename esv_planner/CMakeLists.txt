cmake_minimum_required(VERSION 3.0.2)
project(esv_planner)

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

file(GLOB ESV_PLANNER_IGL_LIBS "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.a")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_conversions roscpp roslib sensor_msgs std_msgs tf2 tf2_ros visualization_msgs
  DEPENDS EIGEN3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

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
  src/occupancy_grid_bridge.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/hybrid_astar.cpp
  src/swept_astar.cpp
  src/svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
  src/se2_svsdf_solver.cpp
  src/implicit_svsdf/grid_map_3d.cpp
  src/implicit_svsdf/pcs_map_manager.cpp
  src/implicit_svsdf/mesh_shape_model.cpp
  src/implicit_svsdf/swept_volume_calculator.cpp
  src/implicit_svsdf/svsdf_midend.cpp
  src/implicit_svsdf/svsdf_backend_optimizer.cpp
  src/implicit_svsdf/debug_publisher.cpp
  src/svsdf_runtime.cpp
)

add_library(esv_planner_core STATIC ${ESV_PLANNER_SOURCES})
target_link_libraries(esv_planner_core
  esv_planner_lmbm
  OpenMP::OpenMP_CXX
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  ${ESV_PLANNER_IGL_LIBS}
)

add_executable(esv_planner_node src/esv_planner_node.cpp)
target_link_libraries(esv_planner_node
  esv_planner_core
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

catkin_install_python(
  PROGRAMS
    test/paper_pipeline_smoke.py
    test/trajectory_generation_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(coarse_to_fine_stages_test
    test/coarse_to_fine_stages_test.cpp
    src/grid_map.cpp
    src/polygon_sdf.cpp
    src/body_frame_sdf.cpp
    src/footprint_model.cpp
    src/collision_checker.cpp
    src/topology_planner.cpp
    src/se2_sequence_generator.cpp
  )
  if(TARGET coarse_to_fine_stages_test)
    target_link_libraries(coarse_to_fine_stages_test
      gtest_main
      ${catkin_LIBRARIES}
      ${EIGEN3_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()
  catkin_add_gtest(planner_geometry_test
    test/planner_geometry_test.cpp
    src/grid_map.cpp
    src/polygon_sdf.cpp
    src/body_frame_sdf.cpp
    src/footprint_model.cpp
    src/collision_checker.cpp
    src/grid_obstacle_source.cpp
    src/swept_astar.cpp
    src/svsdf_evaluator.cpp
  )
  if(TARGET planner_geometry_test)
    target_link_libraries(planner_geometry_test
      gtest_main
      ${catkin_LIBRARIES}
      ${EIGEN3_LIBRARIES}
      ${PCL_LIBRARIES}
    )
  endif()
  add_rostest_gtest(strict_runtime_components_test
    test/strict_runtime_components.test
    test/strict_runtime_components_test.cpp
    src/grid_map.cpp
    src/polygon_sdf.cpp
    src/body_frame_sdf.cpp
    src/footprint_model.cpp
    src/occupancy_grid_bridge.cpp
    src/se2_svsdf_solver.cpp
    src/implicit_svsdf/grid_map_3d.cpp
    src/implicit_svsdf/pcs_map_manager.cpp
    src/implicit_svsdf/mesh_shape_model.cpp
    src/implicit_svsdf/swept_volume_calculator.cpp
    src/implicit_svsdf/svsdf_midend.cpp
    src/implicit_svsdf/svsdf_backend_optimizer.cpp
    src/implicit_svsdf/debug_publisher.cpp
  )
  if(TARGET strict_runtime_components_test)
    target_link_libraries(strict_runtime_components_test
      esv_planner_lmbm
      OpenMP::OpenMP_CXX
      ${catkin_LIBRARIES}
      ${EIGEN3_LIBRARIES}
      ${PCL_LIBRARIES}
      ${ESV_PLANNER_IGL_LIBS}
    )
  endif()
  add_rostest(test/trajectory_generation.test)
  add_rostest(test/paper_pipeline_smoke.test)
  add_rostest(test/strict_svsdf_demo_smoke.test)
endif()
