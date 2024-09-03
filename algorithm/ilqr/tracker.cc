#include "algorithm/ilqr/tracker.h"

#include <iostream>
#include <cmath>

#include "algorithm/math/vec2d.h"
#include "algorithm/math/math_utils.h"
#include "algorithm/math/linear_quadratic_regulator.h"

namespace planning {

bool Tracker::Plan(
    const TrajectoryPoint& start_state,
    const DiscretizedTrajectory& coarse_traj,
    DiscretizedTrajectory* const opt_trajectory) {
  return lqr(start_state, coarse_traj, opt_trajectory);
}

std::pair<Tracker::State, Tracker::State> 
Tracker::CalcaulateInitState(
    const TrajectoryPoint& current_state) {

  double preveiw_x = current_state.x + std::cos(current_state.theta) * 
      current_state.velocity * config_.lateral_config.preview_time;
  double preveiw_y = current_state.y + std::sin(current_state.theta) *
      current_state.velocity * config_.lateral_config.preview_time;

  math::Vec2d cur_xy{preveiw_x, preveiw_y};
  TrajectoryPoint project_pt;
  math::Vec2d proj = follow_trajectory_.GetProjection(cur_xy, &project_pt);
  
  double dx = current_state.x - project_pt.x;
  double dy = current_state.y - project_pt.y;
  double l = std::sin(project_pt.theta) * dx - std::cos(project_pt.theta) *dy;
  
  double theta_error = math::NormalizeAngle(project_pt.theta - current_state.theta);
  State lateral_state{
      l,
      theta_error,
      current_state.delta};

  TrajectoryPoint match_pt = follow_trajectory_.EvaluateTime(current_state.time + 0.0);
  dx = current_state.x - match_pt.x;
  dy = current_state.y - match_pt.y;
  double s = -std::cos(match_pt.theta) * dx - std::sin(match_pt.theta) * dy;
  double v_error = match_pt.velocity - current_state.velocity;
  State longitudinal_state{
      match_pt.s - project_pt.s,
      v_error,
      current_state.a};

  return {lateral_state, longitudinal_state};
}

double Tracker::LateralControl(
    const Tracker::State& state, const double v) {
  double v_amend = std::fmax(2, v);
  double dt = 0.1;
  lateral_A_(0, 1) = v_amend * dt;
  lateral_A_(1, 2) = -v_amend / vehicle_param_.wheel_base * dt;
  
  Eigen::MatrixXd K;
  math::SolveLQRProblem(
      lateral_A_, lateral_B_, 
      lateral_Q_, lateral_R_, 
      config_.tolerance, config_.max_num_iteration,
      &K);
  
  return -(K * state)(0, 0);
}

double Tracker::LongitudinalControl(const Tracker::State& state) {
  Eigen::MatrixXd K;
  math::SolveLQRProblem(
      longitudinal_A_, longitudinal_B_, 
      longitudinal_Q_, longitudinal_R_, 
      config_.tolerance, config_.max_num_iteration,
      &K);

  return -(K * state)(0, 0);
}

TrajectoryPoint Tracker::VehicleDynamic(
    const TrajectoryPoint& cur_state,
    const double delta_rate,
    const double jerk) {

  const double dt = config_.sumulation_dt;
  const double dt_2 = dt / 2.0;
  VehicleState ss_tk1 = vehicle_mode(cur_state.theta, 
                                     cur_state.velocity, 
                                     cur_state.delta,
                                     cur_state.a, 
                                     jerk, 
                                     delta_rate);

  VehicleState ss_tk2 = vehicle_mode(cur_state.theta + ss_tk1.theta * dt_2, 
                                     cur_state.velocity + ss_tk1.v * dt_2, 
                                     cur_state.delta + ss_tk1.delta * dt_2, 
                                     cur_state.a + ss_tk1.a * dt_2, 
                                     jerk, 
                                     delta_rate);

  VehicleState ss_tk3 = vehicle_mode(cur_state.theta + ss_tk2.theta * dt_2, 
                                     cur_state.velocity + ss_tk2.v * dt_2, 
                                     cur_state.delta + ss_tk2.delta * dt_2, 
                                     cur_state.a + ss_tk2.a * dt_2, 
                                     jerk, 
                                     delta_rate);

  VehicleState ss_tk4 = vehicle_mode(cur_state.theta + ss_tk3.theta * dt, 
                                     cur_state.velocity + ss_tk3.v * dt, 
                                     cur_state.delta + ss_tk3.delta * dt, 
                                     cur_state.a + ss_tk3.a * dt, 
                                     jerk, 
                                     delta_rate);

  TrajectoryPoint next_state;
  next_state.time = cur_state.time + dt;
  next_state.x = cur_state.x + (ss_tk1.x + ss_tk2.x * 2.0 + ss_tk3.x * 2.0 + ss_tk4.x) / 6.0 * dt;
  next_state.y = cur_state.y + (ss_tk1.y + ss_tk2.y * 2.0 + ss_tk3.y * 2.0 + ss_tk4.y) / 6.0 * dt;
  next_state.theta = math::NormalizeAngle(cur_state.theta +
                         (ss_tk1.theta + ss_tk2.theta * 2.0 + ss_tk3.theta * 2.0 + ss_tk4.theta) / 6.0 * dt);
  next_state.velocity = std::fmax(0.0, cur_state.velocity + (ss_tk1.v + ss_tk2.v * 2.0 + ss_tk3.v * 2.0 + ss_tk4.v) / 6.0 * dt);
  next_state.delta = math::NormalizeAngle(
                         std::fmin(vehicle_param_.delta_max, 
                                   std::fmax(vehicle_param_.delta_min, 
                                   cur_state.delta + (ss_tk1.delta + ss_tk2.delta * 2.0 + ss_tk3.delta * 2.0 + ss_tk4.delta) / 6.0 * dt)));
  next_state.a = std::fmin(vehicle_param_.max_acceleration,
                           std::fmax(vehicle_param_.min_acceleration, cur_state.a + (ss_tk1.a + ss_tk2.a * 2.0 + ss_tk3.a * 2.0 + ss_tk4.a) / 6.0 * dt));
  next_state.kappa = std::tan(next_state.delta) / vehicle_param_.wheel_base;
  
  double ds = std::hypot(next_state.x - cur_state.x, next_state.y - cur_state.y);
  next_state.s = cur_state.s + ds;
  return next_state;
}

void Tracker::InitMatrix() {
  double dt = config_.dt;

  lateral_A_(0, 0) = 1.0;
  lateral_A_(1, 1) = 1.0;
  lateral_A_(2, 2) = 1.0;

  lateral_B_(2, 0) = 1.0 * dt;
  
  lateral_Q_(0, 0) = lateral_config_.weight_l;
  lateral_Q_(1, 1) = lateral_config_.weight_theta;
  lateral_Q_(2, 2) = lateral_config_.weight_delta;

  lateral_R_(0, 0) = lateral_config_.weight_delta_rate;
  
  longitudinal_A_(0, 0) = 1.0;
  longitudinal_A_(1, 1) = 1.0;
  longitudinal_A_(2, 2) = 1.0;

  longitudinal_A_(0, 1) = dt;
  longitudinal_A_(1, 2) = -dt;
  
  longitudinal_B_(2, 0) = 1.0 * dt;

  longitudinal_Q_(0, 0) = longitudinal_config_.weight_s;
  longitudinal_Q_(1, 1) = longitudinal_config_.weight_v;
  longitudinal_Q_(2, 2) = longitudinal_config_.weight_a;

  longitudinal_R_(0, 0) = longitudinal_config_.weight_j;
}

bool Tracker::lqr(
    const TrajectoryPoint& start_state,
    const DiscretizedTrajectory& coarse_traj,
    DiscretizedTrajectory* const opt_trajectory) {
  follow_trajectory_ = coarse_traj;
  std::vector<TrajectoryPoint> trajectory;

  TrajectoryPoint cur_state = start_state;
  trajectory.push_back(cur_state);
  
  double start_time = follow_trajectory_.trajectory().front().time;
  double end_time = follow_trajectory_.trajectory().back().time;
  cur_state.time = start_time;
  cur_state.s = 0.0;

  int i = 1;
  for (double t = start_time; t < end_time + math::kMathEpsilon; t += config_.sumulation_dt) {
    auto init_state = CalcaulateInitState(cur_state);
    double delta_rate = LateralControl(init_state.first, cur_state.velocity);
    double jerk = LongitudinalControl(init_state.second);
    
    delta_rate = std::fmax(vehicle_param_.delta_rate_min, 
                           std::fmin(vehicle_param_.delta_rate_max, delta_rate));
    jerk = std::fmax(vehicle_param_.jerk_min, 
                     std::fmin(vehicle_param_.jerk_max, jerk));
    trajectory.back().delta_rate = delta_rate;
    trajectory.back().jerk = jerk;

    cur_state = VehicleDynamic(cur_state, delta_rate, jerk);
    cur_state.time = t;
    if (cur_state.time > follow_trajectory_.trajectory().at(i).time - math::kMathEpsilon) {
      trajectory.push_back(cur_state);
      ++i;
    }
  }

  if (trajectory.size() != follow_trajectory_.trajectory().size()) {
    std::cout << "tacker failed." << std::endl;
    return false;
  }

  std::cout << "coarse traj pts size: " << follow_trajectory_.trajectory().size() << std::endl;
  std::cout << "init guess traj pts size: " << trajectory.size() << std::endl;

  *opt_trajectory = DiscretizedTrajectory(trajectory);
  return true;
}

} // namespace planning