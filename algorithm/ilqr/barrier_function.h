#pragma once

#include "algorithm/ilqr/vehicle_model.h"

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>

namespace planning {

template<std::size_t N>
class BarrierFunction {
 public:
  BarrierFunction() = default;

  BarrierFunction(const double param)
      : param_(param) {
    q1_q2_ = q1_ * q2_;
    q1_q2_q2_ = q1_ * q2_ * q2_;
  }

  double value(const double x) {
    // return q1_ * std::exp(q2_ * x);
    double cost = q1_ * std::exp(q2_ * x);
    return cost < q1_ ? 0.0 : cost;
  }

  Eigen::Matrix<double, N, 1> Jacbian(
      const double x, const Eigen::Matrix<double, N, 1>& dx) {
    // return q1_ * q2_ * std::exp(q2_ * x) * dx;
    return q1_q2_ * std::exp(q2_ * x) * dx;
  }
  
  Eigen::Matrix<double, N, N>
  Hessian(
      const double x, 
      const Eigen::Matrix<double, N, 1>& dx, 
      const Eigen::Matrix<double, N, N>& ddx = Eigen::MatrixXd::Zero(N, N)) {
    // return q1_ * q2_ * std::exp(q2_ * x) * ddx + q1_ * q2_ * q2_ * std::exp(q2_ * x) * dx * dx.transpose();
    // return q1_q2_q2_ * std::exp(q2_ * x) * dx * dx.transpose();
    return q1_q2_ * std::exp(q2_ * x) * ddx + q1_q2_q2_ * std::exp(q2_ * x) * dx * dx.transpose();
  }

 private:
  double param_;
  double q1_ = 3.0;
  double q2_ = 2.5;

  double q1_q2_ = 0.0;
  double q1_q2_q2_ = 0.0;
};

} // namespace planning