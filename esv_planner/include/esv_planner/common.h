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

struct PlanningStats {
  double total_solve_time = 0.0;
  double search_time = 0.0;
  double optimization_time = 0.0;
  double min_clearance = -std::numeric_limits<double>::infinity();
  int coarse_path_points = 0;
  int support_points = 0;
  int local_obstacle_points = 0;
  int optimizer_iterations = 0;

  std::vector<double> asVector() const {
    std::vector<double> values;
    values.reserve(8);
    values.push_back(total_solve_time);
    values.push_back(search_time);
    values.push_back(optimization_time);
    values.push_back(min_clearance);
    values.push_back(static_cast<double>(coarse_path_points));
    values.push_back(static_cast<double>(support_points));
    values.push_back(static_cast<double>(local_obstacle_points));
    values.push_back(static_cast<double>(optimizer_iterations));
    return values;
  }
};

// MINCO polynomial piece: 5th-order, C² continuous
// Coefficients for one piece: c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
struct PolyPiece {
  // coeffs[i] = Eigen::Vector2d for x,y; 6 coefficients (order 0..5)
  Eigen::Matrix<double, 2, 6> coeffs = Eigen::Matrix<double, 2, 6>::Zero();
  double duration = 1.0;  // time duration of this piece

  Eigen::Vector2d evaluate(double t) const {
    Eigen::Vector2d p = coeffs.col(5);
    for (int i = 4; i >= 0; --i) {
      p = p * t + coeffs.col(i);
    }
    return p;
  }

  Eigen::Vector2d velocity(double t) const {
    Eigen::Vector2d v = 5.0 * coeffs.col(5);
    for (int i = 4; i >= 1; --i) {
      v = v * t + static_cast<double>(i) * coeffs.col(i);
    }
    return v;
  }

  Eigen::Vector2d acceleration(double t) const {
    Eigen::Vector2d a = 20.0 * coeffs.col(5);
    for (int i = 4; i >= 2; --i) {
      a = a * t + static_cast<double>(i * (i - 1)) * coeffs.col(i);
    }
    return a;
  }
};

// Yaw polynomial piece (1D, 5th-order)
struct YawPolyPiece {
  Eigen::Matrix<double, 1, 6> coeffs = Eigen::Matrix<double, 1, 6>::Zero();
  double duration = 1.0;

  double evaluate(double t) const {
    double val = coeffs(0, 5);
    for (int i = 4; i >= 0; --i) {
      val = val * t + coeffs(0, i);
    }
    return val;
  }

  double velocity(double t) const {
    double val = 5.0 * coeffs(0, 5);
    for (int i = 4; i >= 1; --i) {
      val = val * t + coeffs(0, i) * static_cast<double>(i);
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

  // Sample yaw-rate at global time t
  double sampleYawRate(double t) const {
    double acc = 0.0;
    for (size_t i = 0; i < yaw_pieces.size(); ++i) {
      if (t <= acc + yaw_pieces[i].duration || i + 1 == yaw_pieces.size()) {
        return yaw_pieces[i].velocity(t - acc);
      }
      acc += yaw_pieces[i].duration;
    }
    return 0.0;
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

// --- Shared utility functions (deduplicated from multiple source files) ---

inline std::vector<double> unwrapYawSequence(const std::vector<SE2State>& states) {
  std::vector<double> yaws;
  yaws.reserve(states.size());
  if (states.empty()) return yaws;
  yaws.push_back(states.front().yaw);
  for (size_t i = 1; i < states.size(); ++i) {
    yaws.push_back(yaws.back() + normalizeAngle(states[i].yaw - states[i - 1].yaw));
  }
  return yaws;
}

inline bool pointInPolygon(double px, double py,
                           const std::vector<Eigen::Vector2d>& verts) {
  bool inside = false;
  int n = static_cast<int>(verts.size());
  for (int i = 0, j = n - 1; i < n; j = i++) {
    double yi = verts[i].y(), yj = verts[j].y();
    double xi = verts[i].x(), xj = verts[j].x();
    if (((yi > py) != (yj > py)) &&
        (px < (xj - xi) * (py - yi) / (yj - yi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

inline Eigen::Vector2d rotateIntoBody(const Eigen::Vector2d& world_delta, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * world_delta.x() + s * world_delta.y(),
                         -s * world_delta.x() + c * world_delta.y());
}

inline Eigen::Vector2d rotateIntoWorld(const Eigen::Vector2d& body_vec, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * body_vec.x() - s * body_vec.y(),
                         s * body_vec.x() + c * body_vec.y());
}

inline double waypointChainLength(const std::vector<SE2State>& waypoints) {
  if (waypoints.size() < 2) return 0.0;
  double len = 0.0;
  for (size_t i = 1; i < waypoints.size(); ++i) {
    len += (waypoints[i].position() - waypoints[i - 1].position()).norm();
  }
  return len;
}

// Generic transition clearance: interpolates SE2 states and evaluates via callable
template <typename EvalFn>
double interpolatedClearance(const SE2State& from, const SE2State& to,
                             double sample_step, EvalFn&& eval) {
  const Eigen::Vector2d delta = to.position() - from.position();
  const double len = delta.norm();
  const double yaw_delta = std::abs(normalizeAngle(to.yaw - from.yaw));
  const int linear_steps = std::max(1, static_cast<int>(std::ceil(len / sample_step)));
  const int yaw_steps = std::max(1, static_cast<int>(std::ceil(yaw_delta / 0.10)));
  const int n_steps = std::max(linear_steps, yaw_steps);
  double min_clearance = kInf;
  for (int i = 0; i <= n_steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n_steps);
    SE2State sample;
    sample.x = from.x + delta.x() * t;
    sample.y = from.y + delta.y() * t;
    sample.yaw = normalizeAngle(from.yaw + normalizeAngle(to.yaw - from.yaw) * t);
    min_clearance = std::min(min_clearance, eval(sample));
  }
  return min_clearance;
}

}  // namespace esv_planner
