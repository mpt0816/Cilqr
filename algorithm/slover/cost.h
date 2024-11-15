#pragma once

#include <Eigen/Eigen>

#include "typedefs.h"

namespace ilqr {

class Cost {
 public:
  virtual double Evalute(
    const Eigen::Ref<const Eigen::VetorXd>& x, 
    const Eigen::Ref<const Eigen::VetorXd>& u, 
    const size_t step) = 0;
  
  virtual void Jacobian(
      const Eigen::Ref<const Eigen::VetorXd>& x, 
      const Eigen::Ref<const Eigen::VetorXd>& u, 
      const size_t step,
      Eigen::Ref<Eigen::VectorXd> lx,
      Eigen::Ref<Eigen::VectorXd> lu) = 0;
  
  // for OCP, states and control are independent
  virtual void Hessian(
      const Eigen::Ref<const Eigen::VetorXd>& x,
      const Eigen::Ref<const Eigen::VetorXd>& u, 
      const size_t step,
      Eigen::Ref<Eigen::MatrixXd> lxx,
      Eigen::Ref<Eigen::MatrixXd> luu) = 0;

  virtual void Hessian(
      const Eigen::Ref<const Eigen::VetorXd>& x, 
      const Eigen::Ref<const Eigen::VetorXd>& u, 
      const size_t step,
      Eigen::Ref<Eigen::MatrixXd> lxx,
      Eigen::Ref<Eigen::MatrixXd> luu, 
      Eigen::Ref<Eigen::MatrixXd> lux) = 0;

  virtual ~CostFunction() = default;
}

} // namespace ilqr