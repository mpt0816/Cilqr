
#include "algorithm/ilqr/ilqr_optimizer.h"

#include <cmath>
#include <limits>
#include <ros/ros.h>

#include "algorithm/math/math_utils.h"
#include "algorithm/utils/timer.h"

namespace planning {

IlqrOptimizer::IlqrOptimizer(
    const IlqrConfig& config, const VehicleParam& param,
    const double horizon, const double dt) 
    : config_(config)
    , vehicle_param_(param)
    , horizon_(horizon)
    , delta_t_(dt) {
  tracker_.Init(config_.tracker_config, param);
  vehicle_model_ = VehicleModel(config_, vehicle_param_, horizon_, delta_t_);
  num_of_knots_ = std::floor(horizon_ / delta_t_ + 1);
  State goal = Eigen::MatrixXd::Zero(kStateNum, 1);
  goals_.resize(num_of_knots_, goal);
  CalculateDiscRadius();
  cost_.clear();
}

void IlqrOptimizer::Init(
    const IlqrConfig& config, const VehicleParam& param, 
    const double horizon, const double dt) {
  config_ = config;
  tracker_.Init(config_.tracker_config, param);
  vehicle_param_ = param;
  horizon_ = horizon;
  delta_t_ = dt;
  vehicle_model_ = VehicleModel(config_, vehicle_param_, horizon_, delta_t_);
  num_of_knots_ = std::floor(horizon_ / delta_t_ + 1);
  State goal = Eigen::MatrixXd::Zero(kStateNum, 1);
  goals_.resize(num_of_knots_, goal);
  CalculateDiscRadius();
  cost_.clear();
}
  
bool IlqrOptimizer::Plan(
    const TrajectoryPoint& start_state,
    const DiscretizedTrajectory& coarse_traj,
    const CorridorConstraints& corridor,
    const LaneConstraints& left_lane_cons,
    const LaneConstraints& right_lane_cons,
    DiscretizedTrajectory* const opt_trajectory,
    std::vector<DiscretizedTrajectory>* const iter_trajs) {
  start_state_ = start_state;
  cost_.clear();
  
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

  std::cout << "num_of_knots_: " << num_of_knots_ << std::endl;

  utils::time ilqr_start_time = utils::Time();
  TransformGoals(coarse_traj);

  Optimize(start_state,
           coarse_traj,
           corridor,
           left_lane_cons,
           right_lane_cons,
           opt_trajectory,
           iter_trajs);
  utils::time ilqr_end_time = utils::Time();
  double ilqr_time_cost = utils::Duration(ilqr_start_time, ilqr_end_time);
  std::cout << "ilqr time cost: " << ilqr_time_cost << std::endl;
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
    const DiscretizedTrajectory& coarse_traj,
    std::vector<State>* const guess_state,
    std::vector<Control>* const guess_control) {
  guess_state->resize(num_of_knots_);
  guess_control->resize(num_of_knots_ - 1);

  DiscretizedTrajectory opt_trajectory;
  utils::time tracker_start_time = utils::Time();
  tracker_.Plan(start_state_, coarse_traj, &opt_trajectory);
  utils::time tracker_end_time = utils::Time();
  double tracker_time_cost = utils::Duration(tracker_start_time, tracker_end_time);
  std::cout << "tracker time cost: " << tracker_time_cost << std::endl;

  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    guess_state->at(i) << opt_trajectory.trajectory()[i].x, 
                          opt_trajectory.trajectory()[i].y, 
                          opt_trajectory.trajectory()[i].theta, 
                          opt_trajectory.trajectory()[i].velocity, 
                          opt_trajectory.trajectory()[i].a, 
                          opt_trajectory.trajectory()[i].delta;

    guess_control->at(i) << opt_trajectory.trajectory()[i].jerk,
                            opt_trajectory.trajectory()[i].delta_rate;
  }

  guess_state->back() << opt_trajectory.trajectory().back().x, 
                         opt_trajectory.trajectory().back().y, 
                         opt_trajectory.trajectory().back().theta, 
                         opt_trajectory.trajectory().back().velocity, 
                         opt_trajectory.trajectory().back().a, 
                         opt_trajectory.trajectory().back().delta;
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
  goals_[0] << start_state_.x, start_state_.y, start_state_.theta, start_state_.velocity, 0.0, 0.0;
}

void IlqrOptimizer::Optimize(
    const TrajectoryPoint& start_state,
    const DiscretizedTrajectory& coarse_traj,
    const CorridorConstraints& corridor,
    const LaneConstraints& left_lane_cons,
    const LaneConstraints& right_lane_cons,
    DiscretizedTrajectory* const opt_trajectory,
    std::vector<DiscretizedTrajectory>* const iter_trajs) {

  ShrinkConstraints(corridor, left_lane_cons, right_lane_cons);
  NormalizeHalfPlane();

  std::vector<State> states(num_of_knots_);
  std::vector<Control> controls(num_of_knots_ - 1);
  InitGuess(coarse_traj, &states, &controls);
  iter_trajs->emplace_back(TransformToTrajectory(states, controls));
  
  double cost = TotalCost(states, controls, true);
  std::cout << "init cost: " << cost << std::endl;
  int iter = 0;
  std::vector<Eigen::Matrix<double, kControlNum, kStateNum>> Ks(num_of_knots_ - 1);
  std::vector<Eigen::Matrix<double, kControlNum, 1>> ks(num_of_knots_ - 1);
  std::vector<Eigen::Matrix<double, kControlNum, 1>> Qus;
  std::vector<Eigen::Matrix<double, kControlNum, kControlNum>> Quus;
  while (iter < config_.max_iter_num) {
    std::cout << "ilqr iter: " << (iter + 1) << std::endl;
    Backward(states, controls, &Ks, &ks, &Qus, &Quus);
    Forward(&states, &controls, Ks, ks, Qus, Quus);
    double cost_curr = TotalCost(states, controls, true);
    iter_trajs->emplace_back(TransformToTrajectory(states, controls));
    if (std::fabs(cost - cost_curr) < config_.abs_cost_tol) {
      std::cout << "last cost: " << cost << ", current cost: " << cost_curr << std::endl;
      std::cout << "ilqr converange." << std::endl;
      break;
    }
    cost = cost_curr;
    std::cout << "ilqr iter, cost: " << cost << std::endl;
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
    std::vector<Eigen::Matrix<double, kControlNum, 1>>* const ks,
    std::vector<Eigen::Matrix<double, kControlNum, 1>>* const Qus,
    std::vector<Eigen::Matrix<double, kControlNum, kControlNum>>* const Quus) {
  
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
  Qus->clear();
  Quus->clear();
  for (int i = num_of_knots_ - 2; i >=0; --i) {
    auto Qx = cost_Jx[i] + As[i].transpose() * Vx;
    auto Qu = cost_Ju[i] + Bs[i].transpose() * Vx;

    auto Qxx = cost_Hx[i] + As[i].transpose() * Vxx * As[i];
    auto Quu = cost_Hu[i] + Bs[i].transpose() * Vxx * Bs[i];
    auto Qux = Bs[i].transpose() * Vxx * As[i]; 

    // std::cout << "Qxx: " << Qxx << std::endl;
    // std::cout << "Quu: " << Quu << std::endl;
    // std::cout << "Qux: " << Qux << std::endl;

    // std::cout << "------------------------" << std::endl;

    auto Quu_tem = Quu + rho_ * Eigen::MatrixXd::Identity(kControlNum, kControlNum);

    auto Quu_inv = Quu_tem.inverse();

    Ks->at(i) = -Quu_inv * Qux;
    ks->at(i) = -Quu_inv * Qu;

    Vx = Qx + Ks->at(i).transpose() * Quu * ks->at(i) + Ks->at(i).transpose() * Qu + Qux.transpose() * ks->at(i);
    Vxx = Qxx + Ks->at(i).transpose() * Quu * Ks->at(i) + Ks->at(i).transpose() * Qux + Qux.transpose() * Ks->at(i);

    Qus->push_back(Qu);
    Quus->push_back(Quu);
  }
}

void IlqrOptimizer::Forward(
    std::vector<State>* const states,
    std::vector<Control>* const controls,
    const std::vector<Eigen::Matrix<double, kControlNum, kStateNum>>& Ks,
    const std::vector<Eigen::Matrix<double, kControlNum, 1>>& ks,
    const std::vector<Eigen::Matrix<double, kControlNum, 1>>& Qus,
    const std::vector<Eigen::Matrix<double, kControlNum, kControlNum>>& Quus) {
  
  std::vector<State> new_state = *states;
  std::vector<Control> new_controls = *controls;
  double cost = TotalCost(*states, *controls);
  std::cout << "===============Forward==============" << std::endl;
  std::cout << "old cost: " << cost << std::endl;
  double last_cost = -1e5;
  double alpha = 1.0;
  while (alpha > 1e-8) {
    State x = goals_.front();
    new_state.front() = x;
    for (int i = 0; i < num_of_knots_ - 1; ++i) {
      new_controls[i] = new_controls[i] + Ks[i] * (x - states->at(i)) + alpha * ks[i];
      new_controls[i](1, 0) = math::NormalizeAngle(new_controls[i](1, 0)); 
      // std::cout << "control: " << controls->at(i) << std::endl;
      vehicle_model_.Dynamics(x, new_controls[i], &x);
      new_state.at(i + 1) = x;
    }
    // *states = new_state;

    double new_cost = TotalCost(new_state, *controls);
    std::cout << "new cost: " << new_cost << std::endl;
    double delta_v = 0.0;
    for (int i = 0; i < num_of_knots_ - 1; ++i) {
      delta_v -= (0.5 * alpha * alpha * ks[i].transpose() * Quus[i] * ks[i] + alpha * ks[i].transpose() * Qus[i])(0 ,0);
    }
    
    double z = 0.0;
    double dcost = cost - new_cost;

    // if (delta_v > 0) {
    //   z = dcost / delta_v;
    // } else {
    //   z = (0 < dcost) - (dcost < 0);
    // }

    z = dcost / delta_v;

    if ((delta_v > 0 && z > 1e-4 && z < 10.0) || dcost > 0.0) {
      *controls = new_controls;
      *states = new_state;
      break;
    } else {
      alpha = alpha / 2.0;
      new_state = *states;
      new_controls = *controls;
    }
  }

  if (alpha < 1e-8) {
    rho_ = rho_ * 1.75;
  }

  // std::cout << "after forward cost: " << TotalCost(*states, *controls) << std::endl;
  // std::cout << "sates addr: " << states << std::endl;
  
}

double IlqrOptimizer::TotalCost(
    const std::vector<State>& states,
    const std::vector<Control>& controls,
    const bool log) {
  double j_cost = JCost(states, controls);
  // std::cout << "j_cost: " << j_cost << std::endl;
  double dynamics_cost = DynamicsCost(states, controls);
  // std::cout << "dynamics_cost: " << dynamics_cost << std::endl;
  double corridor_cost = CorridorCost(states);
  // std::cout << "corridor_cost: " << corridor_cost << std::endl;
  double lane_boundary_cost = LaneBoundaryCost(states);
  // std::cout << "lane_boundary_cost: " << lane_boundary_cost << std::endl;
  double total_cost = j_cost + dynamics_cost + corridor_cost + lane_boundary_cost;
  if (log) {
    cost_.push_back(Cost(total_cost, j_cost, dynamics_cost, corridor_cost, lane_boundary_cost));
  }
  
  return total_cost;
  // return j_cost + dynamics_cost;
}

void IlqrOptimizer::ShrinkConstraints(
    const CorridorConstraints& corridor,
    const LaneConstraints& left_lane_cons,
    const LaneConstraints& right_lane_cons) {
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
      double norm = std::hypot(std::hypot(hpoly[0], hpoly[1]), hpoly[2]);
      hpoly = hpoly / norm;
    }
  }

  for (auto& cons : shrinked_left_lane_cons_) {
    auto& hpoly = cons.first;
    double norm = std::hypot(std::hypot(hpoly[0], hpoly[1]), hpoly[2]);
    hpoly = hpoly / norm;
  }

  for (auto& cons : shrinked_right_lane_cons_) {
    auto& hpoly = cons.first;
    double norm = std::hypot(std::hypot(hpoly[0], hpoly[1]), hpoly[2]);
    hpoly = hpoly / norm;
  }
}

double IlqrOptimizer::JCost(
    const std::vector<State>& states,
    const std::vector<Control>& controls) {
  double cost = 0.0;
  for (int i = 0; i < num_of_knots_; ++i) {
    cost += config_.weights.x_target * std::pow((states[i](0, 0) - goals_[i](0, 0)), 2) +
            config_.weights.y_target * std::pow((states[i](1, 0) - goals_[i](1, 0)), 2) +
            config_.weights.theta * std::pow((states[i](2, 0) - goals_[i](2, 0)), 2);
            // config_.weights.v * std::pow((states[i](3, 0) - goals_[i](3, 0)), 2) +
            // config_.weights.a * std::pow((states[i](4, 0) - goals_[i](4, 0)), 2) +
            // config_.weights.delta * std::pow((states[i](5, 0) - goals_[i](5, 0)), 2);
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
  double x_cost = 0.0;
  for (int i = 0; i < num_of_knots_; ++i) {
    x_cost += state_barrier_.value(-states[i](3, 0));
    x_cost += state_barrier_.value(states[i](3, 0) - vehicle_param_.max_velocity);
    x_cost += state_barrier_.value(states[i](4, 0) - vehicle_param_.max_acceleration);
    x_cost += state_barrier_.value(vehicle_param_.min_acceleration - states[i](4, 0));
    x_cost += state_barrier_.value(states[i](5, 0) - vehicle_param_.delta_max);
    x_cost += state_barrier_.value(vehicle_param_.delta_min - states[i](5, 0));
    // std::cout << "states: " << states[i] << std::endl;
    // std::cout << "dynamic v min cost: " << state_barrier_.value(-states[i](3, 0)) << std::endl;
    // std::cout << "dynamic v max cost: " << state_barrier_.value(states[i](3, 0) - vehicle_param_.max_velocity) << std::endl;
    // std::cout << "dynamic a max cost: " << state_barrier_.value(states[i](4, 0) - vehicle_param_.max_acceleration) << std::endl;
    // std::cout << "dynamic a min cost: " << state_barrier_.value(vehicle_param_.min_acceleration - states[i](4, 0)) << std::endl;
    // std::cout << "dynamic delta max cost: " << state_barrier_.value(states[i](5, 0) - vehicle_param_.delta_max) << std::endl;
    // std::cout << "dynamic delta min cost: " << state_barrier_.value(vehicle_param_.delta_min - states[i](5, 0)) << std::endl;
  } 

  // std::cout << "DynamicsCost x: " << x_cost << std::endl;

  double u_cost = 0.0;

  for (int i = 0; i < num_of_knots_ - 1; ++i) {
    u_cost += control_barrier_.value(controls[i](0, 0) - vehicle_param_.jerk_max);
    u_cost += control_barrier_.value(vehicle_param_.jerk_min - controls[i](0, 0));
    u_cost += control_barrier_.value(controls[i](1, 0) - vehicle_param_.delta_rate_max);
    u_cost += control_barrier_.value(vehicle_param_.delta_rate_min - controls[i](1, 0));
  }
  // std::cout << "DynamicsCost u: " << u_cost << std::endl;

  return x_cost + u_cost;
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
      double x = states[i](0, 0) + (L * (j - 0.5) - rf) * std::cos(states[i](2, 0));
      double y = states[i](1, 0) + (L * (j - 0.5) - rf) * std::sin(states[i](2, 0));
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
      double x = states[i](0, 0) + (L * (j - 0.5) - rf) * std::cos(states[i](2, 0));
      double y = states[i](1, 0) + (L * (j - 0.5) - rf) * std::sin(states[i](2, 0));
      
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
              2.0 * config_.weights.y_target * (state(1, 0) - goals_[index](1, 0)),
              2.0 * config_.weights.theta * (state(2, 0) - goals_[index](2, 0)),
              0.0,
              0.0,
              0.0;

  *cost_Ju << 2.0 * config_.weights.jerk * control(0, 0),
              2.0 * config_.weights.delta_rate * control(1, 0);
  
  DynamicsConsJacbian(state, control, cost_Jx, cost_Ju);
  CorridorConsJacbian(index, state, cost_Jx);
  LaneBoundaryConsJacbian(state, cost_Jx);
}

void IlqrOptimizer::CostHessian(
    const int index, const State& state, const Control& control,
    Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx, 
    Eigen::Matrix<double, kControlNum, kControlNum>* const cost_Hu) {
  *cost_Hx << 2.0 * config_.weights.x_target, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 2.0 * config_.weights.y_target, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 2.0 * config_.weights.theta, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 2.0 * config_.weights.v, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 2.0 * config_.weights.a, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 2.0 * config_.weights.delta;

  *cost_Hu << 2.0 * config_.weights.jerk, 0.0,
              0.0, 2.0 * config_.weights.delta_rate;

  DynamicsConsHessian(state, control, cost_Hx, cost_Hu);
  CorridorConsHessian(index, state, cost_Hx);
  LaneBoundaryConsHessian(state, cost_Hx);
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
    double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_sin = (L * (i - 0.5) - rf) * std::sin(state(2, 0));
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
    double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_sin = (L * (i - 0.5) - rf) * std::sin(state(2, 0));
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
    double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_sin = (L * (i - 0.5) - rf) * std::sin(state(2, 0));
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
    double length_cos = (L * (i - 0.5) - rf) * std::cos(state(2, 0));
    double length_sin = (L * (i - 0.5) - rf) * std::sin(state(2, 0));
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
    traj[i].kappa = std::tan(states[i](5, 0)) / vehicle_param_.wheel_base; 
    // std::cout << "TransformToTrajectory delta: " << states[i](5, 0) << std::endl;
    if (i < num_of_knots_ - 1) {
      traj[i].jerk = controls[i](0, 0);
      traj[i].delta_rate = controls[i](1, 0);
    }
  }
  return DiscretizedTrajectory(traj);
}

} // namespace planning