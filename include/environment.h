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

#include <memory>
#include <unordered_map>

#include "math/polygon2d.h"
#include "discretized_trajectory.h"
#include "vehicle_param.h"
#include "planner_config.h"

namespace planning {

class Environment {
 public:
  using DynamicObstacle = std::vector<std::pair<double, math::Polygon2d>>;

  explicit Environment(const PlannerConfig& config) : config_(config) {}

  std::vector<math::Polygon2d>& obstacles() {
    return obstacles_;
  }

  std::vector<DynamicObstacle>& dynamic_obstacles() {
    return dynamic_obstacles_;
  }

  const DiscretizedTrajectory& reference() const {
    return reference_;
  }

  void set_reference(const DiscretizedTrajectory& reference);

  bool CheckCollision(const double time, const math::Box2d& rect);

  bool CheckOptimizationCollision(
      const double time, const math::Pose& pose, 
      const double collision_buffer = 0.0);

  std::unordered_map<int, math::Polygon2d> QueryDynamicObstacles(
      const double time);

  void Visualize();

 private:
  bool CheckStaticCollision(const math::Box2d& rect);

  bool CheckDynamicCollision(const double time, const math::Box2d& rect);

 private:
  PlannerConfig config_;
  std::vector<DynamicObstacle> dynamic_obstacles_;
  std::vector<math::Polygon2d> obstacles_;
  DiscretizedTrajectory reference_;
  std::vector<math::Vec2d> road_barrier_;
};

using Env = std::shared_ptr<Environment>;

} // namespace planning
