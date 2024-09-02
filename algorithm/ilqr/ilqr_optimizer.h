#pragma once

#include <vector>

#include "algorithm/params/planner_config.h"
#include "algorithm/ilqr/corridor.h"
#include "algorithm/utils/discretized_trajectory.h"
#include "algorithm/ilqr/vehicle_model.h"
#include "algorithm/ilqr/barrier_function.h"
#include "algorithm/ilqr/tracker.h"

namespace planning {

class IlqrOptimizer {
 public:
  IlqrOptimizer() = default;    

  IlqrOptimizer(
      const IlqrConfig& config, const VehicleParam& param, 
      const double horizon, const double dt);

  void Init(
      const IlqrConfig& config, const VehicleParam& param, 
      const double horizon, const double dt);
  
  bool Plan(
      const TrajectoryPoint& start_state,
      const DiscretizedTrajectory& coarse_traj,
      const CorridorConstraints& corridor,
      const LaneConstraints& left_lane_cons,
      const LaneConstraints& right_lane_cons,
      DiscretizedTrajectory* const opt_trajectory,
      std::vector<DiscretizedTrajectory>* const iter_trajs);

 private:
  void CalculateDiscRadius();

  void TransformGoals(const DiscretizedTrajectory& coarse_traj);

  void InitGuess(
      const DiscretizedTrajectory& coarse_traj,
      std::vector<State>* const state,
      std::vector<Control>* const control);

  void Optimize(
      const TrajectoryPoint& start_state,
      const DiscretizedTrajectory& coarse_traj,
      const CorridorConstraints& corridor,
      const LaneConstraints& left_lane_cons,
      const LaneConstraints& right_lane_cons,
      DiscretizedTrajectory* const opt_trajectory,
      std::vector<DiscretizedTrajectory>* const iter_trajs);

  void Backward(
      const std::vector<State>& states,
      const std::vector<Control>& controls,
      std::vector<Eigen::Matrix<double, kControlNum, kStateNum>>* const Ks,
      std::vector<Eigen::Matrix<double, kControlNum, 1>>* const ks,
      std::vector<Eigen::Matrix<double, kControlNum, 1>>* const Qus,
    std::vector<Eigen::Matrix<double, kControlNum, kControlNum>>* const Quus);

  void Forward(
      std::vector<State>* const states,
      std::vector<Control>* const controls,
      const std::vector<Eigen::Matrix<double, kControlNum, kStateNum>>& Ks,
      const std::vector<Eigen::Matrix<double, kControlNum, 1>>& ks,
      const std::vector<Eigen::Matrix<double, kControlNum, 1>>& Qus,
      const std::vector<Eigen::Matrix<double, kControlNum, kControlNum>>& Quus);

  double TotalCost(
      const std::vector<State>& states,
      const std::vector<Control>& controls);

  double JCost(
      const std::vector<State>& states,
      const std::vector<Control>& controls);

  double DynamicsCost(
      const std::vector<State>& states,
      const std::vector<Control>& controls);

  double CorridorCost(
      const std::vector<State>& states);

  double LaneBoundaryCost(
      const std::vector<State>& states);

  void CostJacbian(
      const int index, 
      const State& state, const Control& control,
      State* const cost_Jx, Control* cost_Ju);

  void CostHessian(
      const int index, 
      const State& state, const Control& control,
      Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx, 
      Eigen::Matrix<double, kControlNum, kControlNum>* const cost_Hu);

  void ShrinkConstraints(
      const CorridorConstraints& corridor,
      const LaneConstraints& left_lane_cons,
      const LaneConstraints& right_lane_cons);

  Eigen::Vector3d FindNeastLaneSegment(
      const double x, const double y,
      std::vector<std::pair<Eigen::Vector3d, math::LineSegment2d>> lane_segs);

  void DynamicsConsJacbian(
      const State& state, const Control& control,
      State* const cost_Jx, Control* cost_Ju);

  void DynamicsConsHessian(
      const State& state, const Control& control,
      Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx,
      Eigen::Matrix<double, kControlNum, kControlNum>* const cost_Hu);

  void CorridorConsJacbian(
      const int index, 
      const State& state, State* const cost_Jx);

  void CorridorConsHessian(
      const int index, const State& state, 
      Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx);

  void LaneBoundaryConsJacbian(
      const State& state, State* const cost_Jx);

  void LaneBoundaryConsHessian(
      const State& state, 
      Eigen::Matrix<double, kStateNum, kStateNum>* const cost_Hx);

  DiscretizedTrajectory TransformToTrajectory(
      const std::vector<State>& states,
      const std::vector<Control>& controls);

  void NormalizeHalfPlane();

 private:
  double horizon_;
  double delta_t_;
  int num_of_knots_;
  double disc_radius_;

  double rho_ = 1e-10;

  IlqrConfig config_;
  VehicleParam vehicle_param_;

  VehicleModel vehicle_model_;
  BarrierFunction<kStateNum> state_barrier_;
  BarrierFunction<kControlNum> control_barrier_;
  
  TrajectoryPoint start_state_;
  std::vector<State> goals_;
  State init_state_;

  CorridorConstraints shrinked_corridor_;
  LaneConstraints shrinked_left_lane_cons_;
  LaneConstraints shrinked_right_lane_cons_;

  Tracker tracker_;
  
};

} // namespace planning