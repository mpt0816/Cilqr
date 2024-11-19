#pragma once

#include "algorithm/ilqr/vehicle_model.h"

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>

#include "algorithm/math/math_utils.h"
#include "algorithm/math/vec2d.h"

namespace planning {

template<std::size_t N>
class BarrierFunction {
 public:
  BarrierFunction() = default;

  virtual ~BarrierFunction() = default;

  virtual void SetParam(const double t) = 0;

  virtual double GetParam() = 0;

  virtual double value(const double x) = 0;

  virtual Eigen::Matrix<double, N, 1> Jacbian(
      const double x, const Eigen::Matrix<double, N, 1>& dx) = 0;
  
  virtual Eigen::Matrix<double, N, N>
  Hessian(
      const double x, 
      const Eigen::Matrix<double, N, 1>& dx, 
      const Eigen::Matrix<double, N, N>& ddx = Eigen::MatrixXd::Zero(N, N)) = 0;
};

template<std::size_t N>
class ExponentialBarrierFunction : public BarrierFunction<N> {
 public:
  ExponentialBarrierFunction() {
    q1_q2_ = q1_ * q2_;
    q1_q2_q2_ = q1_ * q2_ * q2_;
  }

  double value(const double x) override {
    // return q1_ * std::exp(q2_ * x);
    double cost = q1_ * std::exp(q2_ * x);
    return cost < q1_ ? 0.0 : cost;
  }

  Eigen::Matrix<double, N, 1> Jacbian(
      const double x, const Eigen::Matrix<double, N, 1>& dx) override {
    // return q1_ * q2_ * std::exp(q2_ * x) * dx;
    if (value(x) < math::kMathEpsilon) {
      return 0.0 * q1_q2_ * std::exp(q2_ * x) * dx;
    }
    return q1_q2_ * std::exp(q2_ * x) * dx;
  }
  
  Eigen::Matrix<double, N, N>
  Hessian(
      const double x, 
      const Eigen::Matrix<double, N, 1>& dx, 
      const Eigen::Matrix<double, N, N>& ddx = Eigen::MatrixXd::Zero(N, N)) override {
    // return q1_ * q2_ * std::exp(q2_ * x) * ddx + q1_ * q2_ * q2_ * std::exp(q2_ * x) * dx * dx.transpose();
    // return q1_q2_q2_ * std::exp(q2_ * x) * dx * dx.transpose();
    if (value(x) < math::kMathEpsilon) {
      return 0.0 * (q1_q2_ * std::exp(q2_ * x) * ddx + q1_q2_q2_ * std::exp(q2_ * x) * dx * dx.transpose());
    }
    return q1_q2_ * std::exp(q2_ * x) * ddx + q1_q2_q2_ * std::exp(q2_ * x) * dx * dx.transpose();
  }

 private:
  double q1_ = 0.5;
  double q2_ = 2.5;

  double q1_q2_ = 0.0;
  double q1_q2_q2_ = 0.0;
};

template<std::size_t N>
class RelaxBarrierFunction : public BarrierFunction<N> {
 public:
  RelaxBarrierFunction() {
    reciprocal_t_ = 1.0 / t_;
  }

  void SetParam(const double t) override {
    t_ = t;
    // t_ = std::fmin(20.0, t_);
    // t_ = std::fmax(1.0, t_);
    reciprocal_t_ = 1.0 / t_;
  }

  double GetParam() override {
    return t_;
  }

  void SetEpsilon(const double epsilon) {
    epsilon_ = epsilon;
  }


  double value(const double x) override { 
    if (x < 0.0) {
      return 0.0;
    }
    if (x < -epsilon_) {
      return -reciprocal_t_ * std::log(-x);
    } else {
      return 0.5 * reciprocal_t_ * (std::pow((-x - 2.0 * epsilon_) / epsilon_, 2.0) - 1) - reciprocal_t_ * std::log(epsilon_);
    }
  }

  Eigen::Matrix<double, N, 1> Jacbian(
      const double x, const Eigen::Matrix<double, N, 1>& dx) override {
    if (x < 0.0) {
      return 0 * dx;
    }
    if (x < -epsilon_) {
      return - reciprocal_t_ / x * dx;
    } else {
      return reciprocal_t_ * (x + 2.0 * epsilon_) / epsilon_ / epsilon_ * dx;
    }
  }
  
  Eigen::Matrix<double, N, N>
  Hessian(
      const double x, 
      const Eigen::Matrix<double, N, 1>& dx, 
      const Eigen::Matrix<double, N, N>& ddx = Eigen::MatrixXd::Zero(N, N)) override {
    if (x < 0.0) {
      return Eigen::MatrixXd::Zero(N, N);
    }
    if (x < -epsilon_) {
      return reciprocal_t_ / x / x * dx * dx.transpose() - reciprocal_t_ / x * ddx;
    } else {
      return reciprocal_t_ * (x + 2.0 * epsilon_) / epsilon_ / epsilon_ * dx * dx.transpose();
    }
  }

 private:
  double k_ = 2.0;
  double t_ = 5.0;
  double epsilon_ = 0.08;
  double reciprocal_t_ = 0.0;
};

template<std::size_t N>
class QuadraticBarrierFunction : public BarrierFunction<N> {
 public:  
  QuadraticBarrierFunction() {
    param_ = 1000.0;
  }

  double value(const double x) override {
    if (x < math::kMathEpsilon) {
      return 0.0;
    } else {
      return param_ * std::pow(x, 2.0);
    }
  }

  Eigen::Matrix<double, N, 1> Jacbian(
      const double x, const Eigen::Matrix<double, N, 1>& dx) override {
   if (x < math::kMathEpsilon) {
      return Eigen::MatrixXd::Zero(N, 1);
    } else {
      // std::cout << "Jacbian: " << 2.0 * param_ * dx << std::endl;
      return 2.0 * param_ * dx;
    }
  }
  
  Eigen::Matrix<double, N, N>
  Hessian(
      const double x, 
      const Eigen::Matrix<double, N, 1>& dx, 
      const Eigen::Matrix<double, N, N>& ddx = Eigen::MatrixXd::Zero(N, N)) override {
    if (x < math::kMathEpsilon) {
      return Eigen::MatrixXd::Zero(N, N);
    } else {
      // std::cout << "Hessian: " << 2.0 * param_ * dx * dx.transpose() << std::endl;
      return 2.0 * param_ * dx * dx.transpose();
    }
  }

 private:
  double param_ = 10.0;
};

} // namespace planning