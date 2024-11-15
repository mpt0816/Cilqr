#pragma once

#include <vector>
#include <Eigen/Eigen>

#include "typedefs.h"
#include "function.h"

namespace ilqr {

template <int dim_state, int dim_control>
class KnotPoint {
 public:
  explicit KnotPoint(const int index, const bool is_terminal);

  KnotPoint(const KnotPoint& other) = delete;

  KnotPoint operator=(const KnotPoint& other) = delete;

  KnotPoint(KnotPoint&& other) = delete;

  void SetTimestep(const double dt);

  void SetCost(CostFunction cost_function, CostJacobian cost_jacobian, CostHessian cost_hessian);

  void SetDynamics(DynamicsFunction dynamic_function, DynamicsJacobian dynamic_jacobian);

  void SetConstraint(
      ConstraintFunction constraint_function,
      ConstraintJacobian constraint_jacobian,
      ConstraintHessian constraint_hessian);

  void SetBarrierFunction();

  double CalcCost();

  void CalcCostJacobian();

  void CalcCostHessian();

  void CalcDynamics(double* x_next);

  double CalcConstraintCosts();

  void CalcConstraintJacobians();
  
  void CalcConstraintHessians();
  
 private:
  int knot_point_index = -1;
  double dt_ = 0.0;

  // cost function
  CostFunction cost_function_;
  CostJacobian cost_jacobian_;
  CostHessian cost_hessian_;

  DynamicsFunction dynamic_function_;
  DynamicsJacobian dynamic_jacobian_;

  std::vector<ConstraintFunction> constraint_functions_;
  std::vector<ConstraintJacobian> constraint_jacobians_;
  std::vector<ConstraintHessian> constraint_hessians_;

  // flags
  bool is_terminal_ = false;

  // states and controls
  Eigen::VectorXd x_;
  Eigen::VectorXd u_;

  // backward pass
  Eigen::MatrixXd lxx_;
  Eigen::MatrixXd luu_;
  Eigen::MatrixXd lux_;
  Eigen::VectorXd lx_;
  Eigen::VectorXd lu_;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd B_;

  Eigen::MatrixXd Qxx_;
  Eigen::MatrixXd Quu_;
  Eigen::MatrixXd Qux_;
  Eigen::MatrixXd Qx_;
  Eigen::MatrixXd Qu_;

  Eigen::MatrixXd Qxx_tmp_;
  Eigen::MatrixXd Quu_tmp_;
  Eigen::MatrixXd Qux_tmp_;
  Eigen::MatrixXd Qx_tmp_;
  Eigen::MatrixXd Qu_tmp_;

  Eigen::LLT<Eigen::MatrixXd> Quu_fact_;
  // linear feedback term
  Eigen::MatrixXd K_;
  // feedforward term
  Eigen::VectorXd d_;
  
  // cost to go
  Eigen::MatrixXd P_;
  Eigen::VectorXd p_;

  double delta_V_[2];

};

template <int dim_state, int dim_control>
KnotPoint<dim_state, dim_control>::KnotPoint(
    const int index, const bool is_terminal) 
    : knot_point_index (index)
    , is_terminal_(is_terminal) {
  x_.resize(dim_state);
  u_.resize(dim_control);
  lxx_.resize(dim_state, dim_state);
  luu_.resize(dim_control, dim_control);
  lux_.resize(dim_control, dim_state);
  lx_.resize(dim_state, 1);
  lu_.resize(dim_control, 1);

  A_.resize(dim_state, dim_state);
  B_.resize(dim_state, dim_control);

  Qxx_.resize(dim_state, dim_state);
  Quu_.resize(dim_control, dim_control);
  Qux_.resize(dim_control, dim_state);
  Qx_.resize(dim_state, 1);
  Qu_.resize(dim_control, 1);

  Qxx_tmp_.resize(dim_state, dim_state);
  Quu_tmp_.resize(dim_control, dim_control);
  Qux_tmp_.resize(dim_control, dim_state);
  Qx_tmp_.resize(dim_state, 1);
  Qu_tmp_.resize(dim_control, 1);

  K_.resize(dim_control, dim_state);
  // feedforward term
  d_.resize(dim_control);
  
  // cost to go
  P_.resize(dim_state, dim_state);
  p_.resize(dim_state);

}
  
} // namespace ilqr