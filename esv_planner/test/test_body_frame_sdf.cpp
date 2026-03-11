#include <esv_planner/body_frame_sdf.h>

#include <cmath>
#include <iostream>

using namespace esv_planner;

namespace {

BodyFrameSdf makeTFootprint() {
  BodyFrameSdf sdf;
  sdf.setPolygon({
      {0.25, 0.3}, {0.25, -0.3}, {-0.25, -0.3}, {-0.25, -0.1},
      {-0.6, -0.1}, {-0.6, 0.1}, {-0.25, 0.1}, {-0.25, 0.3}});
  return sdf;
}

bool approxVec(const Eigen::Vector2d& a, const Eigen::Vector2d& b, double tol) {
  return (a - b).norm() <= tol;
}

}  // namespace

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  BodyFrameSdf sdf = makeTFootprint();

  {
    const Eigen::Vector2d p(0.10, 0.25);
    const auto q = sdf.query(p);
    std::cout << "[test] inside_top signed_distance=" << q.signed_distance
              << " gradient=(" << q.gradient.x() << "," << q.gradient.y()
              << ") closest=(" << q.closest_point.x() << ","
              << q.closest_point.y() << ")\n";
    if (!(q.signed_distance < 0.0)) {
      std::cerr << "[test] FAIL: expected interior point to have negative signed distance\n";
      return 1;
    }
    if (!approxVec(q.gradient, Eigen::Vector2d(0.0, 1.0), 1e-3)) {
      std::cerr << "[test] FAIL: expected top interior gradient to point outward (+y)\n";
      return 1;
    }
  }

  {
    const Eigen::Vector2d p(-0.40, 0.20);
    const auto q = sdf.query(p);
    const Eigen::Vector2d expected(0.0, 1.0);
    std::cout << "[test] outside_notch signed_distance=" << q.signed_distance
              << " gradient=(" << q.gradient.x() << "," << q.gradient.y()
              << ") closest=(" << q.closest_point.x() << ","
              << q.closest_point.y() << ")\n";
    if (!(q.signed_distance > 0.0)) {
      std::cerr << "[test] FAIL: expected notch exterior point to have positive signed distance\n";
      return 1;
    }
    if (!approxVec(q.gradient, expected, 1e-3)) {
      std::cerr << "[test] FAIL: expected notch exterior gradient to follow top-arm outward normal\n";
      return 1;
    }
    if (!approxVec(q.closest_point, Eigen::Vector2d(-0.40, 0.10), 1e-3)) {
      std::cerr << "[test] FAIL: expected notch exterior closest point to lie on the arm top edge\n";
      return 1;
    }
  }

  {
    const Eigen::Vector2d p(-0.20, 0.20);
    const auto q = sdf.query(p);
    std::cout << "[test] concave_interior signed_distance=" << q.signed_distance
              << " closest=(" << q.closest_point.x() << "," << q.closest_point.y()
              << ") gradient=(" << q.gradient.x() << "," << q.gradient.y() << ")\n";
    if (!(q.signed_distance < 0.0)) {
      std::cerr << "[test] FAIL: expected concave interior point to remain inside polygon\n";
      return 1;
    }
    if (q.gradient.norm() < 0.9 || q.gradient.norm() > 1.1) {
      std::cerr << "[test] FAIL: expected normalized gradient for concave interior point\n";
      return 1;
    }
  }

  std::cout << "[test] PASS\n";
  return 0;
}
