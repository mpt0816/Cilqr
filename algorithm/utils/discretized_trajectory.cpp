/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#include "algorithm/utils/discretized_trajectory.h"

#include <algorithm>
#include <cmath>

#include "algorithm/math/math_utils.h"

namespace planning {

using math::Vec2d;

DiscretizedTrajectory::DiscretizedTrajectory(
    const DiscretizedTrajectory& rhs, 
    const size_t begin, const size_t end) {
  size_t end_new = end;
  if (end < 0) {
    end_new = rhs.trajectory_.size();
  }
  trajectory_.resize(end_new - begin);
  std::copy_n(std::next(rhs.trajectory_.begin(), begin), trajectory_.size(), trajectory_.begin());
}

DiscretizedTrajectory::DataType::const_iterator
DiscretizedTrajectory::QueryLowerBoundStationPoint(
    const double station) const {
  if (station >= trajectory_.back().s) {
    return trajectory_.end() - 1;
  } else if (station < trajectory_.front().s) {
    return trajectory_.begin();
  }

  return std::lower_bound(
    trajectory_.begin(), trajectory_.end(), station,
    [](const TrajectoryPoint& t, const double station) {
      return t.s < station;
    });
}

DiscretizedTrajectory::DataType::const_iterator 
DiscretizedTrajectory::QueryLowerBoundTimePoint(
    const double time) const {
  if (time >= trajectory_.back().time) {
    return trajectory_.end() - 1;
  } else if (time < trajectory_.front().time) {
    return trajectory_.begin();
  }

  return std::lower_bound(
    trajectory_.begin(), trajectory_.end(), time,
    [](const TrajectoryPoint& tp, const double time) {
      return tp.time < time;
    });
}

TrajectoryPoint LinearInterpolateTrajectory(
    const TrajectoryPoint& p0, const TrajectoryPoint& p1, const double s) {
  double s0 = p0.s;
  double s1 = p1.s;
  if (std::abs(s1 - s0) < math::kMathEpsilon) {
    return p0;
  }

  TrajectoryPoint pt;
  double weight = (s - s0) / (s1 - s0);
  pt.time = (1 - weight) * p0.time + weight * p1.time;
  pt.s = s;
  pt.x = (1 - weight) * p0.x + weight * p1.x;
  pt.y = (1 - weight) * p0.y + weight * p1.y;
  pt.theta = math::slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  pt.kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
  pt.velocity = (1 - weight) * p0.velocity + weight * p1.velocity;
  pt.left_bound = (1 - weight) * p0.left_bound + weight * p1.left_bound;
  pt.right_bound = (1 - weight) * p0.right_bound + weight * p1.right_bound;

  return pt;
}

TrajectoryPoint LinearInterpolateTrajectoryWithTime(
    const TrajectoryPoint& p0, const TrajectoryPoint& p1, const double time) {
  double time0 = p0.time;
  double time1 = p1.time;
  if (std::fabs(time1 - time0) < math::kMathEpsilon) {
    return p0;
  }

  TrajectoryPoint pt;
  double weight = (time - time0) / (time1 - time0);
  pt.time = time;
  pt.s = (1 - weight) * p0.s + weight * p1.s;
  pt.x = (1 - weight) * p0.x + weight * p1.x;
  pt.y = (1 - weight) * p0.y + weight * p1.y;
  pt.theta = math::slerp(p0.theta, p0.time, p1.theta, p1.time, time);
  pt.kappa = (1 - weight) * p0.kappa + weight * p1.kappa;
  pt.velocity = (1 - weight) * p0.velocity + weight * p1.velocity;
  pt.left_bound = (1 - weight) * p0.left_bound + weight * p1.left_bound;
  pt.right_bound = (1 - weight) * p0.right_bound + weight * p1.right_bound;

  return pt;
}

TrajectoryPoint DiscretizedTrajectory::EvaluateStation(
    const double station) const {
  auto iter = QueryLowerBoundStationPoint(station);

  if (iter == trajectory_.begin()) {
    iter = std::next(iter);
  }

  auto prev = std::prev(iter, 1);

  return LinearInterpolateTrajectory(*prev, *iter, station);
}

TrajectoryPoint DiscretizedTrajectory::EvaluateTime(
    const double time) const {
  auto iter = QueryLowerBoundTimePoint(time);

  if (iter == trajectory_.begin()) {
    iter = std::next(iter);
  }

  auto prev = std::prev(iter, 1);

  return LinearInterpolateTrajectoryWithTime(*prev, *iter, time);
}

DiscretizedTrajectory::DataType::const_iterator
DiscretizedTrajectory::QueryNearestPoint(
    const Vec2d& point, double* const out_distance) const {
  auto nearest_iter = trajectory_.begin();
  double nearest_distance = std::numeric_limits<double>::max();

  for (auto iter = trajectory_.begin(); iter != trajectory_.end(); iter++) {
    double dx = iter->x - point.x(), dy = iter->y - point.y();
    double distance = dx * dx + dy * dy;
    if (distance < nearest_distance) {
      nearest_iter = iter;
      nearest_distance = distance;
    }
  }

  if (out_distance != nullptr) {
    *out_distance = sqrt(nearest_distance);
  }
  return nearest_iter;
}

Vec2d DiscretizedTrajectory::GetProjection(
    const Vec2d& xy,
    TrajectoryPoint* const project_point_ptr) const {
  long point_idx = std::distance(trajectory_.begin(), QueryNearestPoint(xy));
  auto project_point = trajectory_[point_idx];
  auto index_start = std::max(0l, point_idx - 1);
  auto index_end = std::min(trajectory_.size() - 1, (ulong) point_idx + 1);

  if (index_start < index_end) {
    double v0x = xy.x() - trajectory_[index_start].x;
    double v0y = xy.y() - trajectory_[index_start].y;

    double v1x = trajectory_[index_end].x - trajectory_[index_start].x;
    double v1y = trajectory_[index_end].y - trajectory_[index_start].y;

    double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);
    double dot = v0x * v1x + v0y * v1y;

    double delta_s = dot / v1_norm;
    project_point = LinearInterpolateTrajectory(
        trajectory_[index_start], trajectory_[index_end], trajectory_[index_start].s + delta_s);
  }

  double nr_x = xy.x() - project_point.x, nr_y = xy.y() - project_point.y;
  double lateral = copysign(hypot(nr_x, nr_y), nr_y * std::cos(project_point.theta) 
                   - nr_x * std::sin(project_point.theta));

  if (project_point_ptr) {
    *project_point_ptr = project_point;
  }
  return {project_point.s, lateral};
}

Vec2d DiscretizedTrajectory::GetCartesian(
    const double station, const double lateral) const {
  auto ref = EvaluateStation(station);
  return {ref.x - lateral * std::sin(ref.theta), ref.y + lateral * std::cos(ref.theta)};
}

} // namespace planning
