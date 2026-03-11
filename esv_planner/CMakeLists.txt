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
  src/hybrid_astar.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(esv_planner_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_planner
  src/test_planner.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_planner
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_hybrid_astar_recovery
  src/test_hybrid_astar_recovery.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/hybrid_astar.cpp
)

target_link_libraries(test_hybrid_astar_recovery
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_hybrid_astar_maze_cases
  src/test_hybrid_astar_maze_cases.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/hybrid_astar.cpp
)

target_link_libraries(test_hybrid_astar_maze_cases
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_topology_shortcut
  src/test_topology_shortcut.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/continuous_svsdf_evaluator.cpp
)

target_link_libraries(test_topology_shortcut
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_topology_body_push
  src/test_topology_body_push.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/continuous_svsdf_evaluator.cpp
)

target_link_libraries(test_topology_body_push
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_topology_fixed_case_clearance
  src/test_topology_fixed_case_clearance.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/continuous_svsdf_evaluator.cpp
)

target_link_libraries(test_topology_fixed_case_clearance
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_motion_sequence
  src/test_motion_sequence.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/continuous_feasibility.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
)

target_link_libraries(test_motion_sequence
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_low_high_semantics
  src/test_low_high_semantics.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/continuous_feasibility.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
)

target_link_libraries(test_low_high_semantics
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_se2_fixed_case_fragmentation
  src/test_se2_fixed_case_fragmentation.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
)

target_link_libraries(test_se2_fixed_case_fragmentation
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_esv_pipeline_maze
  src/test_esv_pipeline_maze.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_esv_pipeline_maze
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_esv_fixed_case_regression
  src/test_esv_fixed_case_regression.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_esv_fixed_case_regression
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_svsdf_phase_consistency
  src/test_svsdf_phase_consistency.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_svsdf_phase_consistency
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_grid_map_esdf
  src/test_grid_map_esdf.cpp
  src/grid_map.cpp
)

target_link_libraries(test_grid_map_esdf
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_geometry_map
  src/test_geometry_map.cpp
  src/grid_map.cpp
)

target_link_libraries(test_geometry_map
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_body_frame_sdf
  src/test_body_frame_sdf.cpp
  src/footprint_model.cpp
)

target_link_libraries(test_body_frame_sdf
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_r2_paper_semantics
  src/test_optimizer_r2_paper_semantics.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_optimizer_r2_paper_semantics
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_se2_time_refinement
  src/test_optimizer_se2_time_refinement.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

add_executable(test_continuous_svsdf_evaluator
  src/test_continuous_svsdf_evaluator.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
)

target_link_libraries(test_continuous_svsdf_evaluator
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

target_link_libraries(test_optimizer_se2_time_refinement
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_select_best_clearance
  src/test_optimizer_select_best_clearance.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_optimizer_select_best_clearance
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_se2_fixed_high_segments
  src/test_optimizer_se2_fixed_high_segments.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_optimizer_se2_fixed_high_segments
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_se2_fixed_large_high_segment
  src/test_optimizer_se2_fixed_large_high_segment.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_optimizer_se2_fixed_large_high_segment
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_stitch_seam_connector
  src/test_stitch_seam_connector.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_stitch_seam_connector
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_se2_maze_t_high_segment
  src/test_optimizer_se2_maze_t_high_segment.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_optimizer_se2_maze_t_high_segment
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_optimizer_se2_maze_l_path1_high_segment
  src/test_optimizer_se2_maze_l_path1_high_segment.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/continuous_svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_optimizer_se2_maze_l_path1_high_segment
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_planner_trigger_state
  src/test_planner_trigger_state.cpp
)

target_link_libraries(test_planner_trigger_state
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(test_trajectory_spatial_sampling
  src/test_trajectory_spatial_sampling.cpp
)

target_link_libraries(test_trajectory_spatial_sampling
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
