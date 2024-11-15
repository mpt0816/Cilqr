#pragma once

#include <Eigen/Eigen>

#include "typedefs.h"

namespace ilqr {

class Dynamics {
 public:
  virtual void Evalute(
    const Eigen::Ref<const Eigen::VetorXd>& x, 
    const Eigen::Ref<const Eigen::VetorXd>& u, 
    const size_t step，
    Eigen::Ref<Eigen::VectorXd> x_next) = 0;
  
  virtual void Jacobian(
      const Eigen::Ref<const Eigen::VetorXd>& x, 
      const Eigen::Ref<const Eigen::VetorXd>& u, 
      const size_t step，
      Eigen::Ref<Eigen::MatrixXd> fx,
      Eigen::Ref<Eigen::MatrixXd> fu) = 0;

  virtual ~Dynamics() = default;
};

} // namespace ilqr