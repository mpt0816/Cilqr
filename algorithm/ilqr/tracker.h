#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>

#include "algorithm/params/planner_config.h"
#include "algorithm/params/vehicle_param.h"
#include "algorithm/utils/discretized_trajectory.h"

namespace planning {

class Tracker {
 public:
  using State = Eigen::Matrix<double, 3, 1>;
  struct VehicleState {
    double x;
    double y;
    double theta; 
    double v; 
    double delta; 
    double a; 
    double j; 
    double delta_rate;
  };

 public:
  Tracker() = default;

  Tracker(const TrackerConfig& config, const VehicleParam& param) 
      : config_(config)
      , vehicle_param_(param) {
    lateral_config_ = config_.lateral_config;
    longitudinal_config_ = config_.longitudinal_config;

    InitMatrix();
  }

  void Init(const TrackerConfig& config, const VehicleParam& param) {
    vehicle_param_ = param;
    config_ = config;
    lateral_config_ = config_.lateral_config;
    longitudinal_config_ = config_.longitudinal_config;

    InitMatrix();
  }

  bool Plan(
      const TrajectoryPoint& start_state,
      const DiscretizedTrajectory& coarse_traj,
      DiscretizedTrajectory* const opt_trajectory);

 private:
  bool lqr(
      const TrajectoryPoint& start_state,
      const DiscretizedTrajectory& coarse_traj,
      DiscretizedTrajectory* const opt_trajectory);

  std::pair<State, State> CalcaulateInitState(
      const TrajectoryPoint& current_state);

  double LateralControl(const State& state, const double v);

  double LongitudinalControl(const State& state);

  TrajectoryPoint VehicleDynamic(
      const TrajectoryPoint& cur_state,
      const double delta_rate,
      const double jerk);

  void InitMatrix();

  VehicleState vehicle_mode(
      const double theta, 
      const double v, 
      const double delta, 
      const double a, 
      const double j,
      const double delta_rate) {
    VehicleState dot_x;
    dot_x.x = v * std::cos(theta);
    dot_x.y = v * std::sin(theta);
    dot_x.theta = v * std::tan(delta) / vehicle_param_.wheel_base;
    dot_x.v = a;
    dot_x.a = j;
    dot_x.delta = delta_rate;
    return dot_x;
  };

 private:
  TrackerConfig config_;
  VehicleParam vehicle_param_;
  
  LateralTrackerConfig lateral_config_;
  LongitudinalTrackerConfig longitudinal_config_;

  DiscretizedTrajectory follow_trajectory_;

  Eigen::Matrix<double, 3, 3> lateral_A_ = Eigen::MatrixXd::Zero(3, 3);
  Eigen::Matrix<double, 3, 1> lateral_B_ = Eigen::MatrixXd::Zero(3, 1);
  Eigen::Matrix<double, 3, 3> lateral_Q_ = Eigen::MatrixXd::Zero(3, 3);
  Eigen::Matrix<double, 1, 1> lateral_R_ = Eigen::MatrixXd::Zero(1, 1);

  Eigen::Matrix<double, 3, 3> longitudinal_A_ = Eigen::MatrixXd::Zero(3, 3);
  Eigen::Matrix<double, 3, 1> longitudinal_B_ = Eigen::MatrixXd::Zero(3, 1);
  Eigen::Matrix<double, 3, 3> longitudinal_Q_ = Eigen::MatrixXd::Zero(3, 3);
  Eigen::Matrix<double, 1, 1> longitudinal_R_ = Eigen::MatrixXd::Zero(1, 1);
};

} // namespace planning