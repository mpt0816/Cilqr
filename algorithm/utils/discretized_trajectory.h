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

#pragma once

#include <utility>
#include <vector>
#include <cassert>

#include "algorithm/math/vec2d.h"

namespace planning {

struct StartState {
  double x, y, theta, v, phi, a, omega;
};

struct TrajectoryPoint {
  double time = 0.0;
  double s = 0.0;

  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  double kappa = 0.0;
  double velocity = 0.0;

  double a = 0.0;
  double jerk = 0.0;
  double delta = 0.0;
  double delta_rate = 0.0;

  double left_bound = 0.0;
  double right_bound = 0.0;
};

typedef std::vector<TrajectoryPoint> Trajectory;

/**
 * Discretized Trajectory
 */
class DiscretizedTrajectory {
 public:
  typedef std::vector<TrajectoryPoint> DataType;

  DiscretizedTrajectory() = default;

  DiscretizedTrajectory(
      const DiscretizedTrajectory& rhs, 
      const size_t begin, 
      const size_t end = -1);

  explicit DiscretizedTrajectory(
      const std::vector<TrajectoryPoint>& points) 
      : trajectory_(std::move(points)) {}

  inline const DataType& trajectory() const { return trajectory_; }

  DataType::const_iterator QueryLowerBoundStationPoint(
      const double station) const;

  DataType::const_iterator QueryLowerBoundTimePoint(
      const double time) const;

  DataType::const_iterator QueryNearestPoint(
      const math::Vec2d& point, double* const out_distance = nullptr) const;

  TrajectoryPoint EvaluateStation(const double station) const;

  TrajectoryPoint EvaluateTime(const double time) const;

  math::Vec2d GetProjection(
      const math::Vec2d& xy,
      TrajectoryPoint* const project_point = nullptr) const;

  math::Vec2d GetCartesian(const double station, const double lateral) const;

  bool empty() const {
    return trajectory_.empty();
  }


 protected:
  std::vector<TrajectoryPoint> trajectory_;
};

} // namespace planning
