#include "esv_planner/true_svsdf_penalty.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace esv_planner {

void TrueSvsdfPenalty::init(const FootprintModel& footprint,
                            const TrueSvsdfPenaltyParams& params) {
  footprint_ = &footprint;
  params_ = params;
}

TrueSvsdfPenalty::Evaluation TrueSvsdfPenalty::evaluate(
    const Trajectory& trajectory,
    int support_count,
    const std::vector<Eigen::Vector2d>& local_obstacles) const {
  Evaluation evaluation;
  evaluation.gradient = Eigen::VectorXd::Zero(
      std::max(0, 2 * (support_count - 2)));
  if (!footprint_ || trajectory.empty() || local_obstacles.empty() || support_count < 2) {
    evaluation.min_clearance = trajectory.empty() ? -kInf : kInf;
    return evaluation;
  }

  const std::vector<SweptSample> samples = buildSamples(trajectory);
  if (samples.empty()) {
    evaluation.min_clearance = -kInf;
    return evaluation;
  }

  const int gradient_dim = static_cast<int>(evaluation.gradient.size());
  const int thread_count =
#ifdef _OPENMP
      std::max(1, omp_get_max_threads());
#else
      1;
#endif
  std::vector<Eigen::VectorXd> thread_gradients(
      static_cast<size_t>(thread_count), Eigen::VectorXd::Zero(gradient_dim));
  std::vector<double> thread_costs(static_cast<size_t>(thread_count), 0.0);
  std::vector<double> thread_clearances(
      static_cast<size_t>(thread_count), std::numeric_limits<double>::infinity());

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (int obstacle_index = 0;
       obstacle_index < static_cast<int>(local_obstacles.size());
       ++obstacle_index) {
    int thread_id = 0;
#ifdef _OPENMP
    thread_id = omp_get_thread_num();
#endif
    const Eigen::Vector2d& obstacle = local_obstacles[static_cast<size_t>(obstacle_index)];
    double best_distance = std::numeric_limits<double>::infinity();
    int best_sample_index = -1;
    Eigen::Vector2d best_gradient = Eigen::Vector2d::Zero();

    for (size_t sample_index = 0; sample_index < samples.size(); ++sample_index) {
      const SweptSample& sample = samples[sample_index];
      const Eigen::Vector2d delta = obstacle - sample.state.position();
      if (delta.norm() > params_.obstacle_cutoff_distance) {
        continue;
      }

      const Eigen::Vector2d body_point =
          rotateIntoBody(delta, sample.state.yaw);
      const BodyFrameQuery query = footprint_->bodyFrameQuery(body_point);
      if (query.signed_distance < best_distance) {
        best_distance = query.signed_distance;
        best_sample_index = static_cast<int>(sample_index);
        best_gradient = rotateIntoWorld(query.gradient, sample.state.yaw);
      }
    }

    if (best_sample_index < 0) {
      continue;
    }

    thread_clearances[static_cast<size_t>(thread_id)] =
        std::min(thread_clearances[static_cast<size_t>(thread_id)], best_distance);
    if (best_distance >= params_.safety_margin) {
      continue;
    }

    const double violation = params_.safety_margin - best_distance;
    const double penalty = params_.obstacle_weight * violation * violation;
    thread_costs[static_cast<size_t>(thread_id)] += penalty;

    const SweptSample& sample = samples[static_cast<size_t>(best_sample_index)];
    const Eigen::Vector2d position_gradient =
        2.0 * params_.obstacle_weight * violation * best_gradient;
    addGradientContribution(sample.segment_index, position_gradient,
                            1.0 - sample.alpha,
                            &thread_gradients[static_cast<size_t>(thread_id)]);
    addGradientContribution(sample.segment_index + 1, position_gradient,
                            sample.alpha,
                            &thread_gradients[static_cast<size_t>(thread_id)]);
  }

  evaluation.min_clearance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < thread_count; ++i) {
    evaluation.cost += thread_costs[static_cast<size_t>(i)];
    evaluation.gradient += thread_gradients[static_cast<size_t>(i)];
    evaluation.min_clearance =
        std::min(evaluation.min_clearance, thread_clearances[static_cast<size_t>(i)]);
  }
  if (!std::isfinite(evaluation.min_clearance)) {
    evaluation.min_clearance = kInf;
  }
  return evaluation;
}

std::vector<TrueSvsdfPenalty::SweptSample> TrueSvsdfPenalty::buildSamples(
    const Trajectory& trajectory) const {
  std::vector<SweptSample> samples;
  double global_time = 0.0;
  for (size_t piece_index = 0; piece_index < trajectory.pos_pieces.size(); ++piece_index) {
    const double duration = trajectory.pos_pieces[piece_index].duration;
    const int steps = std::max(
        2, static_cast<int>(std::ceil(duration / std::max(1e-3, params_.sample_time_step))));
    for (int step = 0; step <= steps; ++step) {
      if (piece_index > 0 && step == 0) {
        continue;
      }
      const double alpha = static_cast<double>(step) / static_cast<double>(steps);
      const double sample_time = global_time + alpha * duration;
      SweptSample sample;
      sample.state = trajectory.sample(sample_time);
      sample.segment_index = static_cast<int>(piece_index);
      sample.alpha = alpha;
      samples.push_back(sample);
    }
    global_time += duration;
  }
  return samples;
}

void TrueSvsdfPenalty::addGradientContribution(
    int support_index,
    const Eigen::Vector2d& gradient,
    double weight,
    Eigen::VectorXd* out_gradient) const {
  if (!out_gradient || support_index <= 0) {
    return;
  }
  const int interior_index = support_index - 1;
  const int offset = 2 * interior_index;
  if (offset + 1 >= out_gradient->size()) {
    return;
  }
  (*out_gradient)(offset) += weight * gradient.x();
  (*out_gradient)(offset + 1) += weight * gradient.y();
}

}  // namespace esv_planner
