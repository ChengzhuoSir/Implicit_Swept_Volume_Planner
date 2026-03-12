cmake_minimum_required(VERSION 3.0.2)
project(esv_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
option(ESV_BUILD_LEGACY_EXPERIMENTAL_TESTS
  "Build retired coefficient-space optimizer experiment tests"
  OFF)

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

set(ESV_MAP_SOURCES
  src/grid_map.cpp
  src/body_frame_sdf.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
)

set(ESV_TOPOLOGY_SOURCES
  ${ESV_MAP_SOURCES}
  src/topology_planner.cpp
  src/continuous_svsdf_evaluator.cpp
)

set(ESV_SEQUENCE_SOURCES
  ${ESV_TOPOLOGY_SOURCES}
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
)

set(ESV_SEQUENCE_WITH_FEASIBILITY_SOURCES
  ${ESV_SEQUENCE_SOURCES}
  src/continuous_feasibility.cpp
)

set(ESV_OPTIMIZER_SOURCES
  ${ESV_SEQUENCE_SOURCES}
  src/trajectory_optimizer.cpp
)

set(ESV_HYBRID_ASTAR_SOURCES
  ${ESV_MAP_SOURCES}
  src/hybrid_astar.cpp
)

set(ESV_SVSDF_SOURCES
  src/grid_map.cpp
  src/footprint_model.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
)

add_executable(esv_planner_node
  src/esv_planner_node.cpp
  ${ESV_OPTIMIZER_SOURCES}
  src/hybrid_astar.cpp
)

target_link_libraries(esv_planner_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_planner
  test/test_planner.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_planner
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_hybrid_astar_recovery
  test/test_hybrid_astar_recovery.cpp
  ${ESV_HYBRID_ASTAR_SOURCES}
)

target_link_libraries(test_hybrid_astar_recovery
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_hybrid_astar_maze_cases
  test/test_hybrid_astar_maze_cases.cpp
  ${ESV_HYBRID_ASTAR_SOURCES}
)

target_link_libraries(test_hybrid_astar_maze_cases
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_topology_shortcut
  test/test_topology_shortcut.cpp
  ${ESV_TOPOLOGY_SOURCES}
)

target_link_libraries(test_topology_shortcut
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_topology_body_push
  test/test_topology_body_push.cpp
  ${ESV_TOPOLOGY_SOURCES}
)

target_link_libraries(test_topology_body_push
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_topology_body_push_no_search
  test/test_topology_body_push_no_search.cpp
  ${ESV_TOPOLOGY_SOURCES}
)

target_link_libraries(test_topology_body_push_no_search
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_topology_fixed_case_clearance
  test/test_topology_fixed_case_clearance.cpp
  ${ESV_TOPOLOGY_SOURCES}
)

target_link_libraries(test_topology_fixed_case_clearance
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_motion_sequence
  test/test_motion_sequence.cpp
  ${ESV_SEQUENCE_WITH_FEASIBILITY_SOURCES}
)

target_link_libraries(test_motion_sequence
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_low_high_semantics
  test/test_low_high_semantics.cpp
  ${ESV_SEQUENCE_WITH_FEASIBILITY_SOURCES}
)

target_link_libraries(test_low_high_semantics
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_sequence_finalize_minimality
  test/test_sequence_finalize_minimality.cpp
  ${ESV_SEQUENCE_WITH_FEASIBILITY_SOURCES}
)

target_link_libraries(test_sequence_finalize_minimality
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_grid_continuous_feasibility_injection
  test/test_grid_continuous_feasibility_injection.cpp
  ${ESV_MAP_SOURCES}
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/continuous_feasibility.cpp
)

target_link_libraries(test_grid_continuous_feasibility_injection
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_sequence_feasibility_injection
  test/test_sequence_feasibility_injection.cpp
  ${ESV_SEQUENCE_WITH_FEASIBILITY_SOURCES}
)

target_link_libraries(test_sequence_feasibility_injection
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_se2_fixed_case_fragmentation
  test/test_se2_fixed_case_fragmentation.cpp
  ${ESV_SEQUENCE_SOURCES}
)

target_link_libraries(test_se2_fixed_case_fragmentation
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_esv_pipeline_maze
  test/test_esv_pipeline_maze.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_esv_pipeline_maze
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_esv_fixed_case_regression
  test/test_esv_fixed_case_regression.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_esv_fixed_case_regression
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_esv_second_case_regression
  test/test_esv_second_case_regression.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_esv_second_case_regression
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_svsdf_phase_consistency
  test/test_svsdf_phase_consistency.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_svsdf_phase_consistency
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_grid_map_esdf
  test/test_grid_map_esdf.cpp
  src/grid_map.cpp
)

target_link_libraries(test_grid_map_esdf
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_geometry_map
  test/test_geometry_map.cpp
  src/grid_map.cpp
)

target_link_libraries(test_geometry_map
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_minco_parameterization
  test/test_minco_parameterization.cpp
  src/minco_parameterization.cpp
)

target_link_libraries(test_minco_parameterization
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_third_party_dependency_probe
  test/test_third_party_dependency_probe.cpp
)

target_link_libraries(test_third_party_dependency_probe
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_body_frame_sdf
  test/test_body_frame_sdf.cpp
  src/body_frame_sdf.cpp
  src/footprint_model.cpp
)

target_link_libraries(test_body_frame_sdf
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_r2_paper_semantics
  test/test_optimizer_r2_paper_semantics.cpp
  ${ESV_SVSDF_SOURCES}
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_optimizer_r2_paper_semantics
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_r2_continuous_polynomial
  test/test_optimizer_r2_continuous_polynomial.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_optimizer_r2_continuous_polynomial
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_se2_time_refinement
  test/test_optimizer_se2_time_refinement.cpp
  ${ESV_SVSDF_SOURCES}
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_optimizer_se2_time_refinement
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_continuous_svsdf_evaluator
  test/test_continuous_svsdf_evaluator.cpp
  ${ESV_SVSDF_SOURCES}
)

target_link_libraries(test_continuous_svsdf_evaluator
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_unified_continuous_evaluator
  test/test_unified_continuous_evaluator.cpp
  src/unified_continuous_evaluator.cpp
  src/body_frame_sdf.cpp
  ${ESV_SVSDF_SOURCES}
)

target_link_libraries(test_unified_continuous_evaluator
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_select_best_clearance
  test/test_optimizer_select_best_clearance.cpp
  ${ESV_SVSDF_SOURCES}
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_optimizer_select_best_clearance
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_se2_fixed_high_segments
  test/test_optimizer_se2_fixed_high_segments.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_optimizer_se2_fixed_high_segments
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_se2_fixed_large_high_segment
  test/test_optimizer_se2_fixed_large_high_segment.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_optimizer_se2_fixed_large_high_segment
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_stitch_seam_connector
  test/test_stitch_seam_connector.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_stitch_seam_connector
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_se2_maze_t_high_segment
  test/test_optimizer_se2_maze_t_high_segment.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_optimizer_se2_maze_t_high_segment
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

if(ESV_BUILD_LEGACY_EXPERIMENTAL_TESTS)
  add_executable(test_optimizer_se2_maze_t_path1_seg3
    test/test_optimizer_se2_maze_t_path1_seg3.cpp
    ${ESV_OPTIMIZER_SOURCES}
  )

  target_link_libraries(test_optimizer_se2_maze_t_path1_seg3
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
  )

  add_executable(test_optimizer_se2_maze_t_path1_seg3_context
    test/test_optimizer_se2_maze_t_path1_seg3_context.cpp
    ${ESV_OPTIMIZER_SOURCES}
  )

  target_link_libraries(test_optimizer_se2_maze_t_path1_seg3_context
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
  )
endif()

add_executable(test_optimizer_se2_maze_l_path1_high_segment
  test/test_optimizer_se2_maze_l_path1_high_segment.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_optimizer_se2_maze_l_path1_high_segment
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_se2_continuous_polynomial
  test/test_optimizer_se2_continuous_polynomial.cpp
  ${ESV_OPTIMIZER_SOURCES}
)

target_link_libraries(test_optimizer_se2_continuous_polynomial
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

if(ESV_BUILD_LEGACY_EXPERIMENTAL_TESTS)
  add_executable(test_optimizer_se2_ct_variables
    test/test_optimizer_se2_ct_variables.cpp
    ${ESV_OPTIMIZER_SOURCES}
  )

  target_link_libraries(test_optimizer_se2_ct_variables
    ${catkin_LIBRARIES}
    ${EIGEN3_LIBRARIES}
  )
endif()

add_executable(test_planner_trigger_state
  test/test_planner_trigger_state.cpp
)

target_link_libraries(test_planner_trigger_state
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_trajectory_spatial_sampling
  test/test_trajectory_spatial_sampling.cpp
)

target_link_libraries(test_trajectory_spatial_sampling
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
