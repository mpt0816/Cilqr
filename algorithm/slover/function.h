#pragma once

#include <functional>

#include "typedefs.h"

namespace ilqr {

class FunctionBase {
 public:
  static constexpr int kNStates = Eigen::Dynamic;
  static constexpr int kNControls = Eigen::Dynamic;

 public:
  virtual ~FunctionBase() = default;

  virtual int StateDimesion() const {
    return 0;
  }

  virtual int ControlDimesion() const {
    return 0;
  }

  virtual void Evalute(
      const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<Eigen::VectorXd> out) = 0;

  virtual void Jacobian(
      const VectorXdRef& x, const VectorXdRef& u, Eigen::Ref<Eigen::VectorXd> jac) = 0;

  virtual void Hessian(
      const VectorXdRef& x, const VectorXdRef& u, const VectorXdRef& b,
      Eigen::Ref<Eigen::VectorXd> hess) = 0;
  
  virtual bool HasHessian() const = 0;

 protected:
  static constexpr double kDefaultTolerance = 1e-4; 
};

using CostFunction = std::function<double(const double* dx, const double* u)>;
using CostJacobian = std::function<void(double* dx, double* du, const double* x, const double* y)>;
using CostHessian = std::function<void(double* ddx, double* ddu, double* dxdu, const double* x, const double* y)>;

using DynamicsFunction = std::function<void(double* x_next, const double* x, const double* u, const double dt)>;
using DynamicsJacobian = std::function<void(double* jac, const double* x, const double* u, const double dt)>;

using ConstraintFunction = std::function<double(const double* x, const double* u)>;
using ConstraintJacobian = std::function<void(double* jac, const double* x, const double* u)>;
using ConstraintHessian = std::function<void(double* ddx, double* ddu, double* dxdu, const double* x, const double* y)>;




  
} // namespace ilqr