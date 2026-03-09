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
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_planner
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

add_executable(benchmark_planner
  src/benchmark_planner.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/collision_checker.cpp
  src/topology_planner.cpp
  src/se2_sequence_generator.cpp
  src/svsdf_evaluator.cpp
  src/trajectory_optimizer.cpp
)

target_link_libraries(benchmark_planner
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
)

target_link_libraries(test_topology_shortcut
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
)

target_link_libraries(test_motion_sequence
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
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_esv_pipeline_maze
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

add_executable(test_optimizer_r2_paper_semantics
  src/test_optimizer_r2_paper_semantics.cpp
  src/grid_map.cpp
  src/footprint_model.cpp
  src/svsdf_evaluator.cpp
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
  src/trajectory_optimizer.cpp
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
  src/trajectory_optimizer.cpp
)

target_link_libraries(test_optimizer_select_best_clearance
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
