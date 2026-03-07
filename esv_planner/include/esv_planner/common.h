#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <limits>

namespace esv_planner {

// SE(2) state: position (x, y) + heading (yaw)
struct SE2State {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  SE2State() = default;
  SE2State(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}

  Eigen::Vector2d position() const { return {x, y}; }

  // Normalize yaw to [-pi, pi]
  void normalizeYaw() {
    yaw = std::atan2(std::sin(yaw), std::cos(yaw));
  }
};

// Grid cell index
struct GridIndex {
  int x = 0;
  int y = 0;

  GridIndex() = default;
  GridIndex(int x_, int y_) : x(x_), y(y_) {}

  bool operator==(const GridIndex& o) const { return x == o.x && y == o.y; }
  bool operator!=(const GridIndex& o) const { return !(*this == o); }
};

// Risk level for trajectory segments
enum class RiskLevel {
  LOW,   // R² optimization sufficient
  HIGH   // SE(2) optimization required
};

// A segment of the SE(2) motion sequence
struct MotionSegment {
  std::vector<SE2State> waypoints;
  RiskLevel risk = RiskLevel::LOW;
};

// MINCO polynomial piece: 5th-order, C² continuous
// Coefficients for one piece: c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
struct PolyPiece {
  // coeffs[i] = Eigen::Vector2d for x,y; 6 coefficients (order 0..5)
  Eigen::Matrix<double, 2, 6> coeffs = Eigen::Matrix<double, 2, 6>::Zero();
  double duration = 1.0;  // time duration of this piece

  Eigen::Vector2d evaluate(double t) const {
    Eigen::Vector2d p = Eigen::Vector2d::Zero();
    double tn = 1.0;
    for (int i = 0; i < 6; ++i) {
      p += coeffs.col(i) * tn;
      tn *= t;
    }
    return p;
  }

  Eigen::Vector2d velocity(double t) const {
    Eigen::Vector2d v = Eigen::Vector2d::Zero();
    double tn = 1.0;
    for (int i = 1; i < 6; ++i) {
      v += coeffs.col(i) * (i * tn);
      tn *= t;
    }
    return v;
  }

  Eigen::Vector2d acceleration(double t) const {
    Eigen::Vector2d a = Eigen::Vector2d::Zero();
    double tn = 1.0;
    for (int i = 2; i < 6; ++i) {
      a += coeffs.col(i) * (i * (i - 1) * tn);
      tn *= t;
    }
    return a;
  }
};

// Yaw polynomial piece (1D, 5th-order)
struct YawPolyPiece {
  Eigen::Matrix<double, 1, 6> coeffs = Eigen::Matrix<double, 1, 6>::Zero();
  double duration = 1.0;

  double evaluate(double t) const {
    double val = 0.0;
    double tn = 1.0;
    for (int i = 0; i < 6; ++i) {
      val += coeffs(0, i) * tn;
      tn *= t;
    }
    return val;
  }

  double velocity(double t) const {
    double val = 0.0;
    double tn = 1.0;
    for (int i = 1; i < 6; ++i) {
      val += coeffs(0, i) * i * tn;
      tn *= t;
    }
    return val;
  }
};

// Full trajectory: piecewise polynomial for (x, y) and yaw
struct Trajectory {
  std::vector<PolyPiece> pos_pieces;
  std::vector<YawPolyPiece> yaw_pieces;

  double totalDuration() const {
    double t = 0.0;
    for (const auto& p : pos_pieces) t += p.duration;
    return t;
  }

  bool empty() const { return pos_pieces.empty(); }

  // Sample SE2 state at global time t
  SE2State sample(double t) const {
    double acc = 0.0;
    for (size_t i = 0; i < pos_pieces.size(); ++i) {
      if (t <= acc + pos_pieces[i].duration || i + 1 == pos_pieces.size()) {
        double local_t = t - acc;
        Eigen::Vector2d p = pos_pieces[i].evaluate(local_t);
        double yaw = 0.0;
        if (i < yaw_pieces.size()) {
          yaw = yaw_pieces[i].evaluate(local_t);
        }
        return SE2State(p.x(), p.y(), yaw);
      }
      acc += pos_pieces[i].duration;
    }
    return SE2State();
  }

  // Sample velocity at global time t
  Eigen::Vector2d sampleVelocity(double t) const {
    double acc = 0.0;
    for (size_t i = 0; i < pos_pieces.size(); ++i) {
      if (t <= acc + pos_pieces[i].duration || i + 1 == pos_pieces.size()) {
        return pos_pieces[i].velocity(t - acc);
      }
      acc += pos_pieces[i].duration;
    }
    return Eigen::Vector2d::Zero();
  }

  // Sample acceleration at global time t
  Eigen::Vector2d sampleAcceleration(double t) const {
    double acc = 0.0;
    for (size_t i = 0; i < pos_pieces.size(); ++i) {
      if (t <= acc + pos_pieces[i].duration || i + 1 == pos_pieces.size()) {
        return pos_pieces[i].acceleration(t - acc);
      }
      acc += pos_pieces[i].duration;
    }
    return Eigen::Vector2d::Zero();
  }
};

struct TopoWaypoint {
  Eigen::Vector2d pos = Eigen::Vector2d::Zero();
  double yaw = 0.0;
  bool has_yaw = false;

  TopoWaypoint() = default;
  explicit TopoWaypoint(const Eigen::Vector2d& pos_) : pos(pos_) {}
  TopoWaypoint(double x, double y) : pos(x, y) {}
  TopoWaypoint(const Eigen::Vector2d& pos_, double yaw_)
      : pos(pos_), yaw(yaw_), has_yaw(true) {}
};

// Candidate path from topology planner
struct TopoPath {
  std::vector<TopoWaypoint> waypoints;
  double length = 0.0;

  void computeLength() {
    length = 0.0;
    for (size_t i = 1; i < waypoints.size(); ++i) {
      length += (waypoints[i].pos - waypoints[i - 1].pos).norm();
    }
  }
};

// Constants
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kInf = std::numeric_limits<double>::infinity();

inline double normalizeAngle(double a) {
  return std::atan2(std::sin(a), std::cos(a));
}

}  // namespace esv_planner
