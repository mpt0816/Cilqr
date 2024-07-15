
#include "ilqr_optimizer.h"

#include <cmath>
#include <limits>
#include <ros/ros.h>

namespace planning {

double NornmalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += 2.0 * M_PI;
  }
  return a;
}

IlqrOptimizer::IlqrOptimizer(
    const IlqrConfig& config, const VehicleParam& param,
    const double horizon, const double dt) 
    : config_(config)
    , vehicle_param_(param)
    , horizon_(horizon)
    , delta_t_(dt) {
  vehicle_model_ = VehicleModel(config_, vehicle_param_, horizon_, delta_t_);
  state_barrier_ = BarrierFunction<kStateNum>(config_.t);
  control_barrier_ = BarrierFunction<kControlNum>(config_.t);
  num_of_knots_ = std::floor(horizon_ / delta_t_ + 1);
  State goal = Eigen::MatrixXd::Zero(kStateNum, 1);
  goals_.resize(num_of_knots_, goal);
  CalculateDiscRadius();
}

void IlqrOptimizer::Init(
    const IlqrConfig& config, const VehicleParam& param, 
    const double horizon, const double dt) {
  config_ = config;
  vehicle_param_ = param;
  horizon_ = horizon;
  delta_t_ = dt;
  vehicle_model_ = VehicleModel(config_, vehicle_param_, horizon_, delta_t_);
  state_barrier_ = BarrierFunction<kStateNum>(config_.t);
  control_barrier_ = BarrierFunction<kControlNum>(config_.t);
  num_of_knots_ = std::floor(horizon_ / delta_t_ + 1);
  State goal = Eigen::MatrixXd::Zero(kStateNum, 1);
  goals_.resize(num_of_knots_, goal);
  CalculateDiscRadius();
}
  
bool IlqrOptimizer::Plan(
    const TrajectoryPoint& start_state,
    const DiscretizedTrajectory& coarse_traj,
    const CorridorConstraints& corridor,
    const LeftLaneConstraints& left_lane_cons,
    const RightLaneConstraints& right_lane_cons,
    DiscretizedTrajectory* const opt_trajectory,
    std::vector<DiscretizedTrajectory>* const iter_trajs) {
  start_state_ = start_state;
  init_state_ << start_state_.x,
                 start_state_.y,
                 start_state_.theta,
                 start_state_.velocity,
                 0.0,
                 0.0;
  
  if (opt_trajectory == nullptr || iter_trajs == nullptr) {
    return false;
  }

  if (corridor.size() == 0 ||
      left_lane_cons.size() == 0 || 
      right_lane_cons.size() == 0) {
    ROS_ERROR("ilqr input constraints error");
    return false;
  }

  if (num_of_knots_ != coarse_traj.trajectory().size()) {
    ROS_ERROR("ilqr input coarse_traj error");
    return false;
  }

  TransformGoals(coarse_traj);

  Optimize(start_state,
           corridor,
           left_lane_cons,
           right_lane_cons,
           opt_trajectory,
           iter_trajs);
}

void IlqrOptimizer::CalculateDiscRadius() {
  int num_of_disc = config_.num_of_disc;
  double length = vehicle_param_.front_hang_length + 
       vehicle_param_.wheel_base + vehicle_param_.rear_hang_length;
  double width = vehicle_param_.width;

  disc_radius_ = std::hypot(width / 2.0, length / 2.0 / num_of_disc);
}

// Too hard to tun parmes.
void IlqrOptimizer::InitGuess(
    std::vector<State>* const guess_state,
    std::vector<Control>* const guess_control) {
  guess_state->resize(num_of_knots_);
  guess_control->resize(num_of_knots_ - 1);
  
  std::vector<Eigen::Matrix<double, kControlNum, kStateNum>> Ks(num_of_knots_ - 1);
  Eigen::Matrix<double, kStateNum, kStateNum> Q = Eigen::MatrixXd::Zero(6, 6);
  Q(0, 0) = 1e-1;
  Q(1, 1) = 1e-1;
  Q(2, 2) = 1e-0;
  Q(3, 3) = 1.0e-6;
  Q(4, 4) = 1.0e-6;
  Q(5, 5) = 1.0e-9;

  std::cout << "Q:" << Q << std::endl;

  Eigen::Matrix<double, kControlNum, kControlNum> R;
  R(0, 0) = 10;
  R(1, 1) = 10;
  Eigen::Matrix<double, kStateNum, kStateNum> P = Q;
  
  SystemMatrix A;
  InputMatrix B;
  Control control;
  control << 0.0, 0.0;
  for (int i = num_of_knots_ - 2; i >= 0; --i) {
    vehicle_model_.DynamicsJacbian(goals_[i], control, &A, &B);
    Ks[i] = (R + B.transpose() * P * B).inverse() * (B.transpose() * P * A);
    P = Q + A.transpose() * P * (A - B * Ks[i]);
  }

  auto clamp = [](const double x, const double min, const double max) {
    return std::fmin(max, std::fmax(x, min));
  };
  
  State x;
  x << 0.0, 0.0, 0.0, 10.0, 0.0, 0.0;
  guess_state->front() = x;
  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    guess_control->at(i) = -Ks[i] * (x - goals_[i]);
    guess_control->at(i)(0, 0) = clamp(guess_control->at(i)(0, 0), vehicle_param_.jerk_min, vehicle_param_.jerk_max);
    guess_control->at(i)(1, 0) = clamp(guess_control->at(i)(1, 0), vehicle_param_.delta_rate_min, vehicle_param_.delta_rate_max);
    vehicle_model_.Dynamics(x, guess_control->at(i), &(guess_state->at(i + 1)));
    x = guess_state->at(i + 1);
  }
}

void IlqrOptimizer::TransformGoals(
    const DiscretizedTrajectory& coarse_traj) {
  State goal;
  goal << 0.0, 0.0, 0.0, 0.0 ,0.0, 0.0;
  goals_.resize(num_of_knots_, goal);
  int i = 0;
  for (const auto& pt : coarse_traj.trajectory()) {
    goals_[i] << pt.x, pt.y, pt.theta, pt.velocity, pt.a, pt.delta;
    ++i;
  }
}

void IlqrOptimizer::Optimize(
    const TrajectoryPoint& start_state,
    const CorridorConstraints& corridor,
    const LeftLaneConstraints& left_lane_cons,
    const RightLaneConstraints& right_lane_cons,
    DiscretizedTrajectory* const opt_trajectory,
    std::vector<DiscretizedTrajectory>* const iter_trajs) {

  ShrinkConstraints(corridor, left_lane_cons, right_lane_cons);
  NormalizeHalfPlane();

  std::vector<State> states(num_of_knots_);
  std::vector<Control> controls(num_of_knots_ - 1);
  InitGuess(&states, &controls);
  iter_trajs->emplace_back(TransformToTrajectory(states, controls));
  
  double cost = TotalCost(states, controls);
  std::cout << "cost: " << cost << std::endl;
  int iter = 0;
  std::vector<Eigen::Matrix<double, kControlNum, kStateNum>> Ks(num_of_knots_ - 1);
  std::vector<Eigen::Matrix<double, kControlNum, 1>> ks(num_of_knots_ - 1);
  while (iter < config_.max_iter_num) {
    std::cout << "ilqr iter: " << (iter + 1) << std::endl;
    Backward(states, controls, &Ks, &ks);
    Forward(&states, &controls, Ks, ks);
    double cost_curr = TotalCost(states, controls);
    iter_trajs->emplace_back(TransformToTrajectory(states, controls));
    if (std::fabs(cost - cost_curr) < config_.abs_cost_tol) {
      break;
    }
    cost = cost_curr;
    std::cout << "cost: " << cost << std::endl;
    ++iter;
  }

  if (iter == config_.max_iter_num) {
    ROS_ERROR("Ilqr Solver Reach Max Iter!");
  }

  ROS_ERROR("Ilqr Solver Finish!");

  *opt_trajectory = TransformToTrajectory(states, controls);
}

void IlqrOptimizer::Backward(
    const std::vector<State>& states,
    const std::vector<Control>& controls,
    std::vector<Eigen::Matrix<double, kControlNum, kStateNum>>* const Ks,
    std::vector<Eigen::Matrix<double, kControlNum, 1>>* const ks) {
  
  std::vector<SystemMatrix> As(num_of_knots_ - 1);
  std::vector<InputMatrix> Bs(num_of_knots_ - 1);

  std::vector<Eigen::Matrix<double, kStateNum, 1>> cost_Jx(num_of_knots_, Eigen::MatrixXd::Zero(kStateNum, 1));
  std::vector<Eigen::Matrix<double, kControlNum, 1>> cost_Ju(num_of_knots_ - 1, Eigen::MatrixXd::Zero(kControlNum, 1));
  std::vector<Eigen::Matrix<double, kStateNum, kStateNum>> cost_Hx(num_of_knots_, Eigen::MatrixXd::Zero(kStateNum, kStateNum));
  std::vector<Eigen::Matrix<double, kControlNum, kControlNum>> cost_Hu(num_of_knots_ - 1, Eigen::MatrixXd::Zero(kControlNum, kControlNum));

  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    vehicle_model_.DynamicsJacbian(states[i], controls[i], &(As[i]), &(Bs[i]));
    CostJacbian(i, states[i], controls[i], &(cost_Jx[i]), &(cost_Ju[i]));
    CostHessian(i, states[i], controls[i], &(cost_Hx[i]), &(cost_Hu[i]));
  }
  Eigen::Matrix<double, kControlNum, 1> temp_Ju = Eigen::MatrixXd::Zero(kControlNum, 1);
  Eigen::Matrix<double, kControlNum, kControlNum> temp_Hu = Eigen::MatrixXd::Zero(kControlNum, kControlNum);
  CostJacbian(num_of_knots_ - 1, states.back(), {0.0, 0.0}, &(cost_Jx.back()), &temp_Ju);
  CostHessian(num_of_knots_ - 1, states.back(), {0.0, 0.0}, &(cost_Hx.back()), &temp_Hu);

  Eigen::Matrix<double, kStateNum, 1> Vx = cost_Jx.back();
  Eigen::Matrix<double, kStateNum, kStateNum> Vxx = cost_Hx.back();

  for (int i = num_of_knots_ - 2; i >=0; --i) {
    auto Qx = cost_Jx[i] + As[i].transpose() * Vx;
    auto Qu = cost_Ju[i] + Bs[i].transpose() * Vx;

    auto Qxx = cost_Hx[i] + As[i].transpose() * Vxx * As[i];
    auto Quu = cost_Hu[i] + Bs[i].transpose() * Vxx * Bs[i] ;
    auto Qux = Bs[i].transpose() * Vxx * As[i]; 

    auto Quu_inv = Quu.inverse();

    Ks->at(i) = -Quu_inv * Qux;
    ks->at(i) = -Quu_inv * Qu;

    Vx = Qx + Ks->at(i).transpose() * Quu * ks->at(i) + Ks->at(i).transpose() * Qu + Qux.transpose() * ks->at(i);
    Vxx = Qxx + Ks->at(i).transpose() * Quu * Ks->at(i) + Ks->at(i).transpose() * Qux + Qux.transpose() * Ks->at(i);
  }
}

void IlqrOptimizer::Forward(
    std::vector<State>* const states,
    std::vector<Control>* const controls,
    const std::vector<Eigen::Matrix<double, kControlNum, kStateNum>>& Ks,
    const std::vector<Eigen::Matrix<double, kControlNum, 1>>& ks) {
  State x = init_state_;
  for (int i = 0; i < num_of_knots_ - 2; ++i) {
    controls->at(i) = controls->at(i) + Ks[i] * (x - states->at(i)) + ks[i]; 
    vehicle_model_.Dynamics(x, controls->at(i), &x);
    states->at(i + 1) = x;
  }
}

double IlqrOptimizer::TotalCost(
    const std::vector<State>& states,
    const std::vector<Control>& controls) {
  double j_cost = JCost(states, controls);
  std::cout << "j_cost: " << j_cost << std::endl;
  // double dynamics_cost = DynamicsCost(states, controls);
  // std::cout << "dynamics_cost: " << dynamics_cost << std::endl;
  // double corridor_cost = CorridorCost(states);
  // std::cout << "corridor_cost: " << corridor_cost << std::endl;
  // double lane_boundary_cost = LaneBoundaryCost(states);
  // std::cout << "lane_boundary_cost: " << lane_boundary_cost << std::endl;
  // return j_cost + dynamics_cost + corridor_cost + lane_boundary_cost;
  return j_cost;
}

void IlqrOptimizer::ShrinkConstraints(
    const CorridorConstraints& corridor,
    const LeftLaneConstraints& left_lane_cons,
    const RightLaneConstraints& right_lane_cons) {
  // shrinked_corridor_ = corridor;
  shrinked_corridor_.resize(corridor.size());
  for (int i = 0; i < corridor.size(); ++i) {
    shrinked_corridor_[i] = corridor[i];
    for (auto& e : shrinked_corridor_[i]) {
      double cccc = e[2];
      e[2] = e[2] - (disc_radius_ + config_.safe_margin) * (e[0] * e[0] + e[1] * e[1]) / std::hypot(e[0], e[1]);
      // std::cout << "hyplane: " << e[0] << "x + " << e[1] << "y + " << e[2] << " < 0" << std::endl;
      // std::cout << "shrink distance: " << (std::fabs(e[2] - cccc) / std::hypot(e[0], e[1])) << std::endl;
    }
    
    // std::cout << "---------------------" << std::endl;
  }

  // std::cout << "radius: " << disc_radius_ << std::endl;

  shrinked_left_lane_cons_.resize(left_lane_cons.size());
  for (int i = 0; i < left_lane_cons.size(); ++i) {
    shrinked_left_lane_cons_[i] = left_lane_cons[i];
    auto& e = shrinked_left_lane_cons_[i].first;
    double cccc = e[2];
    e[2] = e[2] - disc_radius_ * (e[0] * e[0] + e[1] * e[1]) / std::hypot(e[0], e[1]);
    // std::cout << "shrink distance: " << (std::fabs(e[2] - cccc) / std::hypot(e[0], e[1])) << std::endl;
  }

  shrinked_right_lane_cons_.resize(right_lane_cons.size());
  for (int i = 0; i < right_lane_cons.size(); ++i) {
    shrinked_right_lane_cons_[i] = right_lane_cons[i];
    auto& e = shrinked_right_lane_cons_[i].first;
    e[2] = e[2] - disc_radius_ * (e[0] * e[0] + e[1] * e[1]) / std::hypot(e[0], e[1]);
  }
}

void IlqrOptimizer::NormalizeHalfPlane() {

  for (auto& corridor : shrinked_corridor_) {
    for (auto& hpoly : corridor) {
      double norm = std::hypot(hpoly[0], hpoly[1]);
      hpoly = hpoly / norm;
    }
  }

  for (auto& cons : shrinked_left_lane_cons_) {
    auto& hpoly = cons.first;
    double norm = std::hypot(hpoly[0], hpoly[1]);
    hpoly = hpoly / norm;
  }

  for (auto& cons : shrinked_right_lane_cons_) {
    auto& hpoly = cons.first;
    double norm = std::hypot(hpoly[0], hpoly[1]);
    hpoly = hpoly / norm;
  }
}

double IlqrOptimizer::JCost(
    const std::vector<State>& states,
    const std::vector<Control>& controls) {
  double cost = 0.0;
  for (int i = 0; i < num_of_knots_; ++i) {
    cost += config_.weights.x_target * std::pow((states[i](0, 0) - goals_[i](0, 0)), 2) +
            config_.weights.y_target * std::pow((states[i](1, 0) - goals_[i](1, 0)), 2);
  }

  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    cost += config_.weights.jerk * std::pow(controls[i](0, 0), 2) +
            config_.weights.delta_rate * std::pow(controls[i](1, 0), 2);
  }

  return cost;
}

double IlqrOptimizer::DynamicsCost(
     const std::vector<State>& states,
     const std::vector<Control>& controls) {
  double cost = 0.0;
  for (int i = 0; i < num_of_knots_; ++i) {
    cost += state_barrier_.value(-states[i](3, 0));
    cost += state_barrier_.value(states[i](3, 0) - vehicle_param_.max_velocity);
    cost += state_barrier_.value(states[i](4, 0) - vehicle_param_.max_acceleration);
    cost += state_barrier_.value(vehicle_param_.min_acceleration - states[i](4, 0));
    cost += state_barrier_.value(states[i](5, 0) - vehicle_param_.delta_max);
    cost += state_barrier_.value(vehicle_param_.delta_min - states[i](5, 0));
  } 

  // std::cout << "DynamicsCost x: " << cost << std::endl;

  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    cost += control_barrier_.value(controls[i](0, 0) - vehicle_param_.jerk_max);
    cost += control_barrier_.value(vehicle_param_.jerk_min - controls[i](0, 0));
    cost += control_barrier_.value(controls[i](1, 0) - vehicle_param_.delta_rate_max);
    cost += control_barrier_.value(vehicle_param_.delta_rate_min - controls[i](1, 0));
  }

  return cost;
}

double IlqrOptimizer::CorridorCost(
    const std::vector<State>& states) {
  double cost = 0.0;
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;

  for (int i = 0; i < num_of_knots_; ++i) {
    // std::cout << "===============================================================" << std::endl;
    Constraints cons = shrinked_corridor_[i];
    for (int j = 0; j < config_.num_of_disc; ++j) {
      // std::cout << "*************************" << std::endl;
      // double x = states[i](0, 0) + (L * (j - 0.5) - rf) * std::cos(states[i](2, 0));
      // double y = states[i](1, 0) + (L * (j - 0.5) - rf) * std::sin(states[i](2, 0));
      double x = states[i](0, 0);
      double y = states[i](1, 0);
      for (const auto& c : cons) {
        // std::cout << "x, y: " << x << ", " << y << std::endl;
        // std::cout << "plane: " << c[0] << ", " << c[1] << ", " << c[2] << std::endl;
        // std::cout << "g(x) : " << c[0] * x + c[1] * y - c[2] << std::endl;
        // std::cout << (c[0] * x + c[1] * y - c[2] > 0.0 ? "g(x) > 0" : "g(x) < 0") << std::endl;
        // std::cout << "barrier: " << state_barrier_.value(c[0] * x + c[1] * y - c[2]) << std::endl;
        // std::cout << "-----------" << std::endl;
        cost += state_barrier_.value(c[0] * x + c[1] * y - c[2]);
      }
      // std::cout << "*************************" << std::endl;
    }
    // std::cout << "===============================================================" << std::endl;
  }

  return cost;
}

double IlqrOptimizer::LaneBoundaryCost(
    const std::vector<State>& states) {
  double cost = 0.0;
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;

  for (int i = 0; i < num_of_knots_; ++i) {
    for (int j = 0; j < config_.num_of_disc; ++j) {
      // double x = states[i](0, 0) + (L * (j - 0.5) - rf) * std::cos(states[i](2, 0));
      // double y = states[i](1, 0) + (L * (j - 0.5) - rf) * std::sin(states[i](2, 0));
      double x = states[i](0, 0);
      double y = states[i](1, 0);
      
      auto cons_left = FindNeastLaneSegment(x, y, shrinked_left_lane_cons_);
      cost += state_barrier_.value(cons_left[0] * x + cons_left[1] * y - cons_left[2]);
      
      auto cons_right = FindNeastLaneSegment(x, y, shrinked_right_lane_cons_);
      cost += state_barrier_.value(cons_right[0] * x + cons_right[1] * y - cons_right[2]);
    }
  }

  return cost;
}

Eigen::Vector3d IlqrOptimizer::FindNeastLaneSegment(
    const double x, const double y, 
    std::vector<std::pair<Eigen::Vector3d, math::LineSegment2d>> lane_segs) {
  double min_dis = std::numeric_limits<double>::max();
  int min_index = -1;
  for (int i = 0; i < lane_segs.size(); ++i) {
    double dis = lane_segs[i].second.DistanceTo(math::Vec2d(x, y));
    if (dis < min_dis) {
      min_dis = dis;
      min_index = i;
    }
  }
  return lane_segs[min_index].first;
}

void IlqrOptimizer::CostJacbian(
    const int index, const State& state, const Control& control,
    State* const cost_Jx, Control* cost_Ju) {
  *cost_Jx << 2.0 * config_.weights.x_target * (state(0, 0) - goals_[index](0, 0)),
              2.0 * config_.weights.x_target * (state(1, 0) - goals_[index](1, 0)),
              0.0, 0.0, 0.0, 0.0;

  *cost_Ju << 2.0 * config_.weights.jerk * control(0, 0),
              2.0 * config_.weights.delta_rate * control(1, 0);
  
  // DynamicsConsJacbian(state, control, cost_Jx, cost_Ju);
  // CorridorConsJacbian(index, state, cost_Jx);
  // LaneBoundaryConsJacbian(state, cost_Jx);
}

void IlqrOptimizer::CostHessian(
    const int index, const State& state, const Control& control,
    Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx, 
    Eigen::Matrix<double, kControlNum, kControlNum>* const cost_Hu) {
  *cost_Hx << -2.0 * config_.weights.x_target, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, -2.0 * config_.weights.x_target, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  *cost_Hu << 2.0 * config_.weights.jerk, 0.0,
              0.0, 2.0 * config_.weights.delta_rate;

  // DynamicsConsHessian(state, control, cost_Hx, cost_Hu);
  // CorridorConsHessian(index, state, cost_Hx);
  // LaneBoundaryConsHessian(state, cost_Hx);
}

void IlqrOptimizer::DynamicsConsJacbian(
    const State& state, const Control& control,
    State* const cost_Jx, Control* cost_Ju) {
  *cost_Jx += state_barrier_.Jacbian(0.0 - state(3, 0), {0.0, 0.0, 0.0, -1.0, 0.0, 0.0}) +
              state_barrier_.Jacbian(state(3, 0) - vehicle_param_.max_velocity, {0.0, 0.0, 0.0, 1.0, 0.0, 0.0}) +
              state_barrier_.Jacbian(vehicle_param_.min_acceleration - state(4, 0), {0.0, 0.0, 0.0, 0.0, -1.0, 0.0}) +
              state_barrier_.Jacbian(state(4, 0) - vehicle_param_.max_acceleration, {0.0, 0.0, 0.0, 0.0, 1.0, 0.0}) +
              state_barrier_.Jacbian(vehicle_param_.delta_min - state(5, 0), {0.0, 0.0, 0.0, 0.0, 0.0, -1.0}) +
              state_barrier_.Jacbian(state(5, 0) - vehicle_param_.delta_max, {0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

  *cost_Ju += control_barrier_.Jacbian(vehicle_param_.jerk_min - control(0, 0), {-1.0, 0.0}) +
              control_barrier_.Jacbian(control(0, 0) - vehicle_param_.jerk_max, {1.0, 0.0}) +
              control_barrier_.Jacbian(vehicle_param_.delta_rate_min - control(1, 0), {0.0, -1.0}) +
              control_barrier_.Jacbian(control(1, 0) - vehicle_param_.delta_rate_max, {0.0, 1.0});
}

void IlqrOptimizer::DynamicsConsHessian(
    const State& state, const Control& control,
    Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx,
    Eigen::Matrix<double, kControlNum, kControlNum>* const cost_Hu) {
  *cost_Hx += state_barrier_.Hessian(0.0 - state(3, 0), {0.0, 0.0, 0.0, -1.0, 0.0, 0.0}) +
              state_barrier_.Hessian(state(3, 0) - vehicle_param_.max_velocity, {0.0, 0.0, 0.0, 1.0, 0.0, 0.0}) +
              state_barrier_.Hessian(vehicle_param_.min_acceleration - state(4, 0), {0.0, 0.0, 0.0, 0.0, -1.0, 0.0}) +
              state_barrier_.Hessian(state(4, 0) - vehicle_param_.max_acceleration, {0.0, 0.0, 0.0, 0.0, 1.0, 0.0}) +
              state_barrier_.Hessian(vehicle_param_.delta_min - state(5, 0), {0.0, 0.0, 0.0, 0.0, 0.0, -1.0}) +
              state_barrier_.Hessian(state(5, 0) - vehicle_param_.delta_max, {0.0, 0.0, 0.0, 0.0, 0.0, 1.0});

  *cost_Hu += control_barrier_.Hessian(vehicle_param_.jerk_min - control(0, 0), {-1.0, 0.0}) +
              control_barrier_.Hessian(control(0, 0) - vehicle_param_.jerk_max, {1.0, 0.0}) +
              control_barrier_.Hessian(vehicle_param_.delta_rate_min - control(1, 0), {0.0, -1.0}) +
              control_barrier_.Hessian(control(1, 0) - vehicle_param_.delta_rate_max, {0.0, 1.0});
}

void IlqrOptimizer::CorridorConsJacbian(
    const int index, const State& state, State* const cost_Jx) {
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;
  Constraints cons = shrinked_corridor_[index];

  for (int i = 0; i < config_.num_of_disc; ++i) {
    // double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    // double length_sin = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_cos = 0;
    double length_sin = 0;
    double x = state(0, 0) + length_cos;
    double y = state(1, 0) + length_sin;

    for (const auto& c : cons) {
      *cost_Jx += state_barrier_.Jacbian(c[0] * x + c[1] * y - c[2], {c[0], c[1], -c[0] * length_sin + c[1] * length_cos, 0.0, 0.0, 0.0});
    }
  }
}

void IlqrOptimizer::CorridorConsHessian(
    const int index, const State& state, 
    Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx) {
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;
  
  Constraints cons = shrinked_corridor_[index];
  Eigen::Matrix<double, kStateNum, kStateNum> ddx = Eigen::MatrixXd::Zero(kStateNum, kStateNum);
  for (int i = 0; i < config_.num_of_disc; ++i) {
    // double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    // double length_sin = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_cos = 0;
    double length_sin = 0;
    double x = state(0, 0) + length_cos;
    double y = state(1, 0) + length_sin;

    for (const auto& c : cons) {
      ddx(2, 2) = -c[0] * length_cos - c[1] * length_sin;
      *cost_Hx += state_barrier_.Hessian(c[0] * x + c[1] * y - c[2], {c[0], c[1], -c[0] * length_sin + c[1] * length_cos, 0.0, 0.0, 0.0}, ddx);
    }
  }
}

void IlqrOptimizer::LaneBoundaryConsJacbian(
    const State& state, State* const cost_Jx) {
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;

  for (int i = 0; i < config_.num_of_disc; ++i) {
    // double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    // double length_sin = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_cos = 0;
    double length_sin = 0;
    double x = state(0, 0) + length_cos;
    double y = state(1, 0) + length_sin;
      
    auto cons_left = FindNeastLaneSegment(x, y, shrinked_left_lane_cons_);
    *cost_Jx += state_barrier_.Jacbian(cons_left[0] * x + cons_left[1] * y - cons_left[2], {cons_left[0], cons_left[1], -cons_left[0] * length_sin + cons_left[1] * length_cos, 0.0, 0.0, 0.0});
      
    auto cons_right = FindNeastLaneSegment(x, y, shrinked_right_lane_cons_);
    *cost_Jx += state_barrier_.Jacbian(cons_right[0] * x + cons_right[1] * y - cons_right[2], {cons_right[0], cons_right[1], -cons_right[0] * length_sin + cons_right[1] * length_cos, 0.0, 0.0, 0.0});
  }
}

void IlqrOptimizer::LaneBoundaryConsHessian(
    const State& state, 
    Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx) {
  double L = (vehicle_param_.rear_hang_length + vehicle_param_.wheel_base + vehicle_param_.front_hang_length) / config_.num_of_disc;
  double rf = vehicle_param_.rear_hang_length;
  Eigen::Matrix<double, kStateNum, kStateNum> ddx = Eigen::MatrixXd::Zero(kStateNum, kStateNum);

  for (int i = 0; i < config_.num_of_disc; ++i) {
    // double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    // double length_sin = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_cos = 0;
    double length_sin = 0;
    double x = state(0, 0) + length_cos;
    double y = state(1, 0) + length_sin;
    
    auto cons_left = FindNeastLaneSegment(x, y, shrinked_left_lane_cons_);
    ddx(2, 2) = -cons_left[0] * length_cos - cons_left[1] * length_sin;
    *cost_Hx += state_barrier_.Hessian(cons_left[0] * x + cons_left[1] * y - cons_left[2], {cons_left[0], cons_left[1], -cons_left[0] * length_sin + cons_left[1] * length_cos, 0.0, 0.0, 0.0}, ddx);
      
    auto cons_right = FindNeastLaneSegment(x, y, shrinked_right_lane_cons_);
    ddx(2, 2) = -cons_right[0] * length_cos - cons_right[1] * length_sin;
    *cost_Hx += state_barrier_.Hessian(cons_right[0] * x + cons_right[1] * y - cons_right[2], {cons_right[0], cons_right[1], -cons_right[0] * length_sin + cons_right[1] * length_cos, 0.0, 0.0, 0.0}, ddx);
  }
}

DiscretizedTrajectory IlqrOptimizer::TransformToTrajectory(
    const std::vector<State>& states,
    const std::vector<Control>& controls) {
  std::vector<TrajectoryPoint> traj(num_of_knots_);
  for (int i = 0; i < num_of_knots_; ++i) {
    traj[i].time = i * delta_t_;
    traj[i].x = states[i](0, 0);
    traj[i].y = states[i](1, 0);
    traj[i].theta = states[i](2, 0);
    traj[i].velocity = states[i](3, 0);
    traj[i].a = states[i](4, 0);
    traj[i].delta = states[i](5, 0);
    if (i < num_of_knots_ - 1) {
      traj[i].jerk = controls[i](0, 0);
      traj[i].delta_rate = controls[i](1, 0);
    }
  }
  return DiscretizedTrajectory(traj);
}

} // namespace planning