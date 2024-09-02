
#include "algorithm/ilqr/vehicle_model.h"

#include <cmath>
#include <iostream>

namespace planning {

double VehicleModel::NornmalizeAngle(const double angle) {
  // double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  // if (a < 0.0) {
  //   a += 2.0 * M_PI;
  // }
  return angle;
}

VehicleModel::VehicleModel(
    const IlqrConfig& config,
    const VehicleParam& param, 
    const double horizon, 
    const double dt)
    : config_(config)
    , param_(param)
    , horizon_(horizon)
    , delta_t_(dt) {}

void VehicleModel::DynamicsJacbian(
    const State& state, const Control& control,
    SystemMatrix* const A, InputMatrix* const B) {
  double L = param_.wheel_base;
  // Euler Method is too poor.
  // double theta = state(2, 0);
  // double delta = state(5, 0);
  // double v = state(3, 0);

  // *A << 1.0, 0.0, -std::sin(theta) * v * delta_t_, std::cos(theta) * delta_t_, 0.0, 0.0,
  //       0.0, 1.0, std::cos(theta) * v * delta_t_, std::sin(theta) * delta_t_, 0.0, 0.0,
  //       0.0, 0.0, 1.0, std::tan(delta) / L * delta_t_, 0.0, v * delta_t_ * (1.0 + std::pow(std::tan(delta), 2)) / L,
  //       0.0, 0.0, 0.0, 1.0, delta_t_, 0.0,
  //       0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
  //       0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  // *B << 0.0, 0.0,
  //       0.0, 0.0,
  //       0.0, 0.0,
  //       0.0, 0.0,
  //       delta_t_, 0.0,
  //       0.0, delta_t_;

  double v = state(3, 0);
  double theta = NornmalizeAngle(state(2, 0));
  double delta = NornmalizeAngle(state(5, 0));
  
  double a = state(4, 0);
  double jerk = control(0, 0);
  double delta_rate = control(1, 0);
  
  double theta_mid = theta + 0.5 * delta_t_ * v * std::tan(delta) / L;
  double tan_delta = std::tan(delta);
  double tan_delta_rate = std::tan(delta + 0.5 * delta_t_ * delta_rate);
  double cos_theta_mid = std::cos(theta_mid); 
  double sin_theta_mid = std::sin(theta_mid);
  double tan_delta_square = tan_delta * tan_delta;
  double tan_delta_rate_square = tan_delta_rate * tan_delta_rate;
  double v_tan_delta_rate =  v * (tan_delta_rate_square + 1);

  *A << 1.0, 0.0, -delta_t_ * (0.5 * a * delta_t_ + v) * sin_theta_mid, 
             delta_t_ * cos_theta_mid - 0.5 * delta_t_ * delta_t_ * (0.5 * a * delta_t_ + v) * sin_theta_mid * tan_delta / L, 
             0.5 * delta_t_ * delta_t_ * cos_theta_mid, 
             -0.5 * delta_t_ * delta_t_ * v * (0.5 * a * delta_t_ + v) * (tan_delta_square + 1) * sin_theta_mid / L,
             
        0.0, 1.0, delta_t_ * (0.5 * a * delta_t_ + v) * cos_theta_mid,
             delta_t_ * sin_theta_mid + 0.5 * delta_t_ * delta_t_ * (0.5 * a * delta_t_ + v) * cos_theta_mid * tan_delta / L,
             0.5 * delta_t_ * delta_t_ * sin_theta_mid,
             0.5 * delta_t_ * delta_t_ * v * (0.5 * a * delta_t_ + v) * (tan_delta_square + 1) * cos_theta_mid / L,
             
        0.0, 0.0, 1.0, 
             delta_t_ * tan_delta_rate / L, 
             0.5 * delta_t_ *  delta_t_ * tan_delta_rate / L, 
             delta_t_ * v_tan_delta_rate / L,
             
        0.0, 0.0, 0.0, 1.0, delta_t_, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  *B << 0.0, 0.0,
        0.0, 0.0, 
        0.0, 0.5 * delta_t_ * delta_t_ * v * (tan_delta_rate_square + 1) / L,
        0.5 * delta_t_ * delta_t_, 0.0,
        delta_t_, 0.0,
        0.0, delta_t_;
}

void VehicleModel::Dynamics(
    const State& state, const Control& control,
    State* const next_state) {
  // SystemMatrix A;
  // InputMatrix B;
  // DynamicsJacbian(state, control, &A, &B);
  // *next_state = A * state + B * control;

  // State inc;
  // inc << state(3, 0) * std::cos(state(2, 0)) * delta_t_,
  //        state(3, 0) * std::sin(state(2, 0)) * delta_t_,
  //        state(3, 0) * std::tan(state(2, 0)) / param_.wheel_base * delta_t_,
  //        state(4, 0) * delta_t_,
  //        control(0, 0) * delta_t_,
  //        control(1, 0) * delta_t_;
  // *next_state = state + inc;
  
  double theta = state(2, 0);
  double v = state(3, 0);
  double a = state(4, 0);
  double delta = state(5, 0);
  
  State k1 = DynamicsContinuous(state, control);
  State mid_state = state + 0.5 * delta_t_ * k1;
  State k2 = DynamicsContinuous(mid_state, control);
  *next_state = state + delta_t_ * k2;
}

State VehicleModel::DynamicsContinuous(
    const State& state, const Control& control) {
  double theta = NornmalizeAngle(state(2, 0));
  double v = state(3, 0);
  double a = state(4, 0);
  double delta = NornmalizeAngle(state(5, 0));
  
  State res;
  res << v * std::cos(theta),
         v * std::sin(theta),
         v * std::tan(delta) / param_.wheel_base,
         a,
         control(0, 0),
         control(1, 0);
  return res;
}

} // namespace planning