#pragma once

#include <isweep_planner/core/common.h>
#include <isweep_planner/core/geometry_map.h>
#include <nav_msgs/OccupancyGrid.h>
#include <Eigen/Dense>
#include <vector>

namespace isweep_planner {

class GridMap {
public:
  GridMap();

  // Initialize from ROS OccupancyGrid
  void fromOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  // Coordinate conversions
  GridIndex worldToGrid(double wx, double wy) const;
  Eigen::Vector2d gridToWorld(int gx, int gy) const;

  // Queries
  bool isInside(int gx, int gy) const;
  bool isOccupied(int gx, int gy) const;
  bool isRawOccupied(int gx, int gy) const;
  double getEsdf(double wx, double wy) const;
  double getEsdfCell(int gx, int gy) const;
  int linearIndex(int gx, int gy) const { return gy * width_ + gx; }

  // Inflate occupancy grid by given radius (in meters)
  // Compute Euclidean Signed Distance Field
  void computeEsdf();

  // Accessors
  double resolution() const { return resolution_; }
  int width() const { return width_; }
  int height() const { return height_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }
  bool ready() const { return ready_; }

  const std::vector<int8_t>& occupancy() const { return occupancy_; }
  const std::vector<double>& esdf() const { return esdf_; }
  const std::vector<int8_t>& inflated() const { return inflated_; }
  const GeometryMap& geometryMap() const { return geometry_map_; }

private:
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.05;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  bool ready_ = false;

  std::vector<int8_t> occupancy_;   // raw occupancy (-1=unknown, 0=free, 100=occupied)
  std::vector<int8_t> inflated_;    // inflated occupancy
  std::vector<double> esdf_;        // Euclidean signed distance field
  GeometryMap geometry_map_;
};

}  // namespace isweep_planner
