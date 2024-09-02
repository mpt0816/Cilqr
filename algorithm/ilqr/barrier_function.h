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
      : param_(param) {}

  double value(const double x) {
    return q1_ * std::exp(q2_ * x);
  }

  Eigen::Matrix<double, N, 1> Jacbian(
      const double x, const Eigen::Matrix<double, N, 1>& dx) {
    return q1_ * q2_ * std::exp(q2_ * x) * dx;
  }
  
  Eigen::Matrix<double, N, N>
  Hessian(
      const double x, 
      const Eigen::Matrix<double, N, 1>& dx, 
      const Eigen::Matrix<double, N, N>& ddx = Eigen::MatrixXd::Zero(N, N)) {
    return q1_ * q2_ * std::exp(q2_ * x) * ddx + q1_ * q2_ * q2_ * std::exp(q2_ * x) * dx * dx.transpose();
  }

 private:
  double param_;
  double q1_ = 0.5;
  double q2_ = 2.5;
};

} // namespace planning