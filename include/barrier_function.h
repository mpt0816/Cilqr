#pragma once

#include "vehicle_model.h"

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
    // std::cout << "input: " << x << std::endl;
    // std::cout << "output: " << q1_ * std::exp(q2_ * x) << std::endl;
    // return -1.0 / param_  * std::log(-x);
    return q1_ * std::exp(q2_ * x);
  }

  Eigen::Matrix<double, N, 1> Jacbian(
      const double x, const Eigen::Matrix<double, N, 1>& dx) {
    // return -1.0 / param_ * dx / (x + 1e-10);
    return q1_ * q2_ * std::exp(q2_ * x) * dx;
  }
  
  Eigen::Matrix<double, N, N>
  Hessian(
      const double x, 
      const Eigen::Matrix<double, N, 1>& dx, 
      const Eigen::Matrix<double, N, N>& ddx = Eigen::MatrixXd::Zero(N, N)) {
    // return -1.0 / param_ * (x * ddx - dx * dx.transpose()) / (x + 1e-10) / (x + 1e-10);
    return q1_ * q2_ * std::exp(q2_ * x) * ddx + q1_ * q2_ * q2_ * std::exp(q2_ * x) * dx * dx.transpose();
  }

 private:
  double param_;
  double q1_ = 0.5;
  double q2_ = 2.5;
};

} // namespace planning