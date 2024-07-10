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

#include "planner_config.h"

#include "dp_planner.h"
#include "corridor.h"

namespace planning {

class TrajectoryPlanner {
public:
  struct StartState {
    double x, y, theta, v, phi, a, omega;
  };

  explicit TrajectoryPlanner(const PlannerConfig& config, const Env& env)
    : config_(config), dp_(config, env), corridor_(config.corridor_config, env) {}

  bool Plan(const StartState& state, DiscretizedTrajectory& result);


  ConvexPolygons SafeCorridors() {
    return convex_polygons_;
  }

  std::vector<std::vector<math::Vec2d>> points_for_corridors() {
    return corridor_.points_for_corridors();
  }

  std::vector<math::LineSegment2d> left_lane_boundary() {
    return left_lane_boundary_;
  }

  std::vector<math::LineSegment2d> right_lane_boundary() {
    return right_lane_boundary_;
  }

private:
  PlannerConfig config_;
  DpPlanner dp_;
  Corridor corridor_;
  ConvexPolygons convex_polygons_;

  std::vector<math::LineSegment2d> left_lane_boundary_;
  std::vector<math::LineSegment2d> right_lane_boundary_;
};

} // namespace planning