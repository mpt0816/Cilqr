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

#include <tuple>
#include <limits>
#include <cmath>
#include <memory>

#include "algorithm/math/pose.h"
#include "algorithm/math/vec2d.h"
#include "algorithm/math/polygon2d.h"
#include "algorithm/utils/environment.h"
#include "algorithm/params/planner_config.h"

namespace planning {

constexpr int NT = 5;
constexpr int NS = 7;
constexpr int NL = 10;

class DpPlanner {
 public:
  DpPlanner(const PlannerConfig& config, const Env& env);

  bool Plan(
      const double start_x, const double start_y, 
      const double start_theta, DiscretizedTrajectory &result);

 private:
  struct StateCell {
    double cost = std::numeric_limits<double>::max();
    double current_s =  std::numeric_limits<double>::min();
    int parent_s_ind = -1;
    int parent_l_ind = -1;

    StateCell() = default;

    StateCell(double cost, double cur_s, int parent_s_ind, int parent_l_ind)
      : cost(cost), current_s(cur_s)
      , parent_s_ind(parent_s_ind), parent_l_ind(parent_l_ind) {}
  };

  struct StateIndex {
    int t = -1, s = -1, l = -1;

    StateIndex() = default;

    StateIndex(int tt, int ss, int ll) : t(tt), s(ss), l(ll) {}
  };

  struct StartState {
    double start_s = 0;
    double start_l = 0;
    double start_theta = 0;
  };

  Env env_;
  PlannerConfig config_;

  double unit_time_;
  std::array<double, NT> time_;
  std::array<double, NS> station_;
  std::array<double, NL - 1> lateral_;

  StartState state_;
  StateCell state_space_[NT][NS][NL];

  double safe_margin_;

  double GetCollisionCost(StateIndex parent_l_ind, StateIndex cur_ind);

  std::pair<double, double> GetCost(StateIndex parent_ind, StateIndex cur_ind);

  double GetLateralOffset(double s, int l_ind) {
    if (l_ind == NL - 1) return 0.0;

    auto ref = env_->reference().EvaluateStation(s);
    double lb = -ref.right_bound + safe_margin_;
    double ub = ref.left_bound - safe_margin_;

    return lb + (ub - lb) * lateral_[l_ind];
  }

  std::vector<math::Vec2d> InterpolateLinearly(
      double parent_s, int parent_l_ind, 
      int cur_t_ind, int cur_s_ind, int cur_l_ind);
};


} // namespace planning
