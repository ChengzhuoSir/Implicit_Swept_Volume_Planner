#pragma once

#include <isweep_planner/env/polygon_sdf.h>

#include <Eigen/Core>
#include <limits>
#include <vector>

namespace isweep_planner {

struct BodyFrameQuery {
  double signed_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector2d closest_point = Eigen::Vector2d::Zero();
  Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
  double winding_number = 0.0;
  bool inside = false;
};

class BodyFrameSdf {
public:
  BodyFrameSdf() = default;

  void setPolygon(const std::vector<Eigen::Vector2d>& vertices);

  const std::vector<Eigen::Vector2d>& vertices() const { return vertices_; }

  double signedDistance(const Eigen::Vector2d& point) const;

  BodyFrameQuery query(const Eigen::Vector2d& point) const;
  std::vector<BodyFrameQuery> queryBatch(
      const Eigen::Ref<const Eigen::MatrixXd>& points) const;

private:
  struct NearestEdgeResult {
    double distance = std::numeric_limits<double>::infinity();
    size_t edge_index = 0;
    Eigen::Vector2d closest_point = Eigen::Vector2d::Zero();
  };

  std::vector<Eigen::Vector2d> vertices_;

  NearestEdgeResult nearestEdge(const Eigen::Vector2d& point) const;
  Eigen::Vector2d outwardNormal(size_t edge_index) const;
};

}  // namespace isweep_planner
