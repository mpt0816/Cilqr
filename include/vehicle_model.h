#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "planner_config.h"

namespace planning {

constexpr int kStateNum = 6;
constexpr int kControlNum = 2;

using State = Eigen::Matrix<double, kStateNum, 1>;
using Control = Eigen::Matrix<double, kControlNum, 1>;
using SystemMatrix = Eigen::Matrix<double, kStateNum, kStateNum>;
using InputMatrix = Eigen::Matrix<double, kStateNum, kControlNum>;

class VehicleModel {
 public:
  VehicleModel() = default;

  VehicleModel(
    const IlqrConfig& config,
    const VehicleParam& param,
    const double horizon, 
    const double dt);


  void DynamicsJacbian(
      const State& state, const Control& control,
      SystemMatrix* const A, InputMatrix* const B);

  void Dynamics(
      const State& state, const Control& control,
      State* const next_state);

  
 private:
  double NornmalizeAngle(const double angle);

  State DynamicsContinuous(
      const State& state, const Control& control);

 private:
  double horizon_;
  double delta_t_;
  int num_of_knots_;
  IlqrConfig config_;
  VehicleParam param_;
  double k = 0.0001;

};
  
} // namespace planning