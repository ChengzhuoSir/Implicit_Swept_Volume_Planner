#pragma once

#include <esv_planner/common.h>
#include <esv_planner/grid_map.h>

#include <Eigen/Core>

#include <vector>

namespace esv_planner {

class PathSparsifier {
public:
  PathSparsifier() = default;

  void init(const GridMap& map,
            double line_clearance,
            double max_segment_length,
            int max_support_points);

  std::vector<SE2State> sparsify(const std::vector<Eigen::Vector2d>& dense_path,
                                 const SE2State& start,
                                 const SE2State& goal) const;

private:
  bool segmentSafe(const Eigen::Vector2d& a, const Eigen::Vector2d& b) const;
  void assignYaw(std::vector<SE2State>* states,
                 const SE2State& start,
                 const SE2State& goal) const;

  const GridMap* map_ = nullptr;
  double line_clearance_ = 0.0;
  double max_segment_length_ = 1.0;
  int max_support_points_ = 12;
};

}  // namespace esv_planner
