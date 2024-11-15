#pragma once

#include <vector>
#include <algorithm>

#include "typedefs.h"
#include "function.h"
#include "knot_point.h"
#include "cost_function.h"
#include "dynamics.h"

namespace ilqr {

template <int dim_state, int dim_control>
class iLQR {
 public:
  explicit iLQR(const int horizon_length, const double dt);

  ~Solver() = default;

  void ResizeHorize(const int horizon_length);

  void SetProblem(
      const CostFunction& cost_function,
      const Dynamics& dynamics);

  void SetControlBound(
      const Eigen::VectorXd& u_min, Eigen::VectorXd& u_max);

  void SetOptions(const Options& options);

  SolverStatus Solve(
      const Eigen::VectorXd& x_0, const VectorXdList& u_list);

 private:
  void OpenLoopRollout(
      const Eigen::VectorXd& x_0, const VectorXdList& u_list,
      Eigen::Ref<Eigen::VetorXd> x_list);

  void CalcDynamicsJacobian();

  void CalcCostFuncJacobian();

  void CalcCostFuncHessian();

  void CalcCost();
  
  bool BackwardPass(const double reg);

  void ForwardPass(const double alpha);

  double CalGradientNorm();

 private:
  Solver(const Solver& other) = delete;

  Solver(Solver&& other) = delete;

  Solver& operator=(const Solver& other) = delete;

  Solver& operator=(Solver&& other) = delete;


 private:
  int horizon_length_ = 0;
  double dt_ = 0.0;
  std::vector<KnotPoint> knots_;

  CostFunction cost_function_;
  Dynamics dynamics_;
  
  bool is_set_control_bound_ = false;
  Eigen::VectorXd u_min_;
  Eigen::VectorXd u_max_;

  Eigen::VectorXd u_zore_;
  
  Options options_;
  SolverStatus solver_status_;
  
  Eigen::VectorXd x0_;
  VectorXdList x_list_;
  VectorXdList u_list_;
  VectorXdList x_old_list_; 
  VectorXdList u_old_list_; 
  
  MatrixXdList fx_list_;
  MatrixXdList fu_list_;
  VectorXdList lx_list_;
  VectorXdList lu_list_;
  MatrixXdList lxx_list_;
  MatrixXdList luu_list_;
  // 对于OCP，在cost中，x和u一般都是不相关的
  MatrixXdList lux_list_;
  
  MatrixXdList P_list_;
  VectorXdList p_list_;

  MatrixXdList K_list_;
  VectorXdList d_list_;


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
  Eigen::VectorXd Qx_;
  Eigen::VectorXd Qu_;

  Eigen::MatrixXd Qux_reg_;
  Eigen::MatrixXd QuuF_;

  Eigen::LLT<Eigen::MatrixXd> Quu_fact_;
  // linear feedback term
  Eigen::MatrixXd K_;
  // feedforward term
  Eigen::VectorXd d_;
  
  // cost to go
  Eigen::MatrixXd P_;
  Eigen::VectorXd p_;

  double delta_V_[2];

  static std::vector<double> alpha_list_{1.0000, 0.5012, 0.2512, 0.1259, 0.0631, 0.0316, 0.0158, 0.0079, 0.0040, 0.0020, 0.0010}; 
};

template <int dim_state, int dim_control>
iLQR<dim_state, dim_control>::iLQR(const int horizon_length) 
    : horizon_length_(horizon_length)
    , dt_(dt)
    , is_set_control_bound_(false)
    , options_()
    , solver_status_() {
  x0_.resize(dim_state);
  u_min_.resize(dim_control);
  u_max_.resize(dim_control);

  u_zore_.resize(dim_control);
  u_zore_.setZero();

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
  Qx_.resize(dim_state);
  Qu_.resize(dim_control);

  Qux_reg_.resize(dim_control, dim_state);
  QuuF_.resize(dim_control, dim_control);

  K_.resize(dim_control, dim_state);
  // feedforward term
  d_.resize(dim_control);
  
  // cost to go
  P_.resize(dim_state, dim_state);
  p_.resize(dim_state);
  
  // resize vector
  x_list_.resize(horizon_length_ + 1);
  u_list_.resize(horizon_length_);

  fx_list_.resize(horizon_length_);
  fu_list_.resize(horizon_length_);
  lx_list_.resize(horizon_length_ + 1);
  lu_list_.resize(horizon_length_ + 1);
  lxx_list_.resize(horizon_length_ + 1);
  luu_list_.resize(horizon_length_ + 1);
  lux_list_.resize(horizon_length_ + 1);
  
  P_list_.resize(horizon_length_ + 1);
  p_list_.resize(horizon_length_ + 1);

  K_list_.resize(horizon_length_);
  d_list_.resize(horizon_length_);

  std::fill(x_list_.begin(), x_list_.end(), Eigen::VectorXd::Zero(dim_state));
  std::fill(u_list_.begin(), u_list_.end(), Eigen::VectorXd::Zero(dim_control));
  std::fill(fx_list_.begin(), fx_list_.end(), Eigen::MatrixXd::Zero(dim_state, dim_state));
  std::fill(fu_list_.begin(), fu_list_.end(), Eigen::MatrixXd::Zero(dim_state, dim_control));
  std::fill(lx_list_.begin(), lx_list_.end(), Eigen::VectorXd::Zero(dim_state));
  std::fill(lu_list_.begin(), lu_list_.end(), Eigen::VectorXd::Zero(dim_control));
  std::fill(lxx_list_.begin(), lxx_list_.end(), Eigen::MatrixXd::Zero(dim_state, dim_state));
  std::fill(luu_list_.begin(), luu_list_.end(), Eigen::MatrixXd::Zero(dim_control, dim_control));
  std::fill(lux_list_.begin(), lux_list_.end(), Eigen::MatrixXd::Zero(dim_control, dim_state));
  std::fill(P_list_.begin(), P_list_.end(), Eigen::MatrixXd::Zero(dim_state, dim_state));
  std::fill(p_list_.begin(), p_list_.end(), Eigen::VectorXd::Zero(dim_state));
  std::fill(K_list_.begin(), K_list_.end(), Eigen::MatrixXd::Zero(dim_control, dim_state));
  std::fill(d_list_.begin(), d_list_.end(), Eigen::VectorXd::Zero(dim_control));

  Eigen::initParallel();
}

template <int dim_state, int dim_control>
void iLQR<dim_state, dim_control>::ResizeHorize(
    const int horizon_length) {
  horizon_length_ = horizon_length;

  // resize vector
  xs_list_.resize(horizon_length_ + 1);
  u_list_.resize(horizon_length_);

  fx_list_.resize(horizon_length_);
  fu_list_.resize(horizon_length_);
  lx_list_.resize(horizon_length_ + 1);
  lu_list_.resize(horizon_length_ + 1);
  lxx_list_.resize(horizon_length_ + 1);
  luu_list_.resize(horizon_length_ + 1);
  lux_list_.resize(horizon_length_ + 1);
  
  P_list_.resize(horizon_length_ + 1);
  p_list_.resize(horizon_length_ + 1);

  K_list_.resize(horizon_length_);
  d_list_.resize(horizon_length_);
}

template <int dim_state, int dim_control>
void iLQR<dim_state, dim_control>::SetControlBound(
    const Eigen::VectorXd& u_min, Eigen::VectorXd& u_max) {
  is_set_control_bound_ = true;
  u_min_ = u_min;
  u_max_ = u_max;
}

template <int dim_state, int dim_control>
void iLQR<dim_state, dim_control>::SetOptions(const Options& options) {
  options_ = options;
}

template <int dim_state, int dim_control>
void iLQR<dim_state, dim_control>::SetProblem(
      const CostFunction& cost_function,
      const Dynamics& dynamics) {
  cost_function_ = cost_function;
  dynamics_ = dynamics_;
}

template <int dim_state, int dim_control>
SolverStatus iLQR<dim_state, dim_control>::Solve(
    const Eigen::VectorXd& x0, const VectorXdList& u_list) {
  auto sgn = [](const double value) {
    return (0.0 < value) - (value < 0.0);
  };

  x0_ = x0; u_list_ = u_list;
  OpenLoopRollout(x0, u_list_, x_list_);
  double cost_old = CalcCost();

  bool is_forward_pass_updated = true;

  double dcost = 0.0;
  double lambda = 1.0;
  double dlambda = 1.0;
  double z = 0.0;
  double cost_new = 0.0;

  int iter = 0;
  for (iter = 0; iter < options_.iter_max; ++iter) {
    x_old_list_ = x_list_; u_old_list_ = u_list_;

    if (is_forward_pass_updated) {
      CalcDynamicsJacobian();
      CalcCostFuncJacobian();
      CalcCostFuncHessian();
      is_forward_pass_updated = false;
    }

    bool is_backward_pass_done = false;
    while (!is_backward_pass_done) {
      bool is_diverge = BackwardPass(lambda);
      // backward pass发散, 增大正则化系数，重新进行 backward pass
      if (is_diverge) {
        dlambda = std::fmax(dlambda * options_.regularization_ratio, options_.regularization_ratio);
        lambda = std::fmax(lambda * dlambda, options_.regularization_min);
        if (lambda > options_.regularization_max) {
          return SolverStatus::kUnsolved;
        } else {
          continue;
        }
      }
      is_backward_pass_done = true;
    }
    
    // 控制量的梯度归一化，并取归一化的最大值，如果梯度太小，则终止迭代
    double gnorm = CalGradientNorm();
    if (gnorm < options_.gradient_norm_min && lambda < 1e-5) {
      return SolverStatus::kSuccess;
    }

    bool is_forward_pass_done = false;
    double alpha = 0.0;
    if (is_backward_pass_done) {
      for (int i = 0; i < alpha_list_.size(); ++i) {
        alpha = alpha_list_[i];
        // update x_list_ and u_list_ in ForwardPass
        ForwardPass(alpha);
        cost_new = CalcCost();
        dcost = cost_old - cost_new;
        double expected = -alpha * (delta_V_[0] + alpha * delta_V_[1]);
        
        z = dcost / expected;
        if (z > options_.beta_min && z < options_.beta_max) {
          is_forward_pass_done = true;
          break;
        }
        // 需要改变 alpha 的值重新进行 forward pass
        x_list_ = x_old_list_;
        u_list_ = u_old_list_;
      }

      if(!is_forward_pass_done) {
        alpha = 0.0;
      }
    } 

    if (is_forward_pass_done) {
      dlambda = std::fmin(dlambda / options_.regularization_ratio, 1.0 / options_.regularization_ratio);
      // 保证 lambda 大于最小值
      lambda = lambda * dlambda * (lambda > options_.regularization_min);

      is_forward_pass_updated = true;
      
      // ilqr 迭代收敛
      if (dcost < options_.tol_abs || 
          std::fabs(dcost / cost_old) < options_.tol_rel) {
          cost_old = cost_new;
        return SolverStatus::kSuccess;
      }
      cost_old = cost_new;
    } else {
      dlambda = std::fmax(dlambda * options_.regularization_ratio, options_.regularization_ratio);
      lambda = std::fmax(lambda * dlambda, options_.regularization_min);
      
      // ilqr 迭代发散
      if (lambda > options_.regularization_max) {
        return SolverStatus::kUnsolved;
      }
    }
  }
  return SolverStatus::kMaxIterations;
}

template <int dim_state, int dim_control>
void iLQR<dim_state, dim_control>::OpenLoopRollout(
    const Eigen::VectorXd& x_0, const VectorXdList& u_list,
    Eigen::Ref<Eigen::VetorXd> x_list) {
  x_list[0] = x_0;
  for (int k = 1; k <= horizon_length_; ++k) {
    dynamics_.Evalute(x_list[k - 1], u_list[k - 1], x_list[k]);
  }
}

template <int dim_state, int dim_control>
void iLQR<dim_state, dim_control>::CalcDynamicsJacobian() {
  for (int k = 0; k < horizon_length_; ++k) {
    dynamics_.Jacobian(x_list_[k], u_list_[k], fx_list_[k], fu_list_[k]);
  }
}

template <int dim_state, int dim_control>
void iLQR<dim_state, dim_control>::CalcCostFuncJacobian() {
  for (int k = 0; k < horizon_length_; ++k) {
    cost_function_.Jacobian(x_list_[k], u_list_[k], lx_list_[k], lu_list_[k]);
  }
  cost_function_.Jacobian(x_list_[horizon_length_], u_zore_, 
                          lx_list_[horizon_length_], lu_list_[horizon_length_]);
  lu_list_.back().setZero();
}

template <int dim_state, int dim_control>
void iLQR<dim_state, dim_control>::CalcCostFuncHessian() {
  for (int k = 0; k < horizon_length_; ++k) {
    cost_function_.Hessian(x_list_[k], u_list_[k], lxx_list_[k], luu_list_[k]);
  }
  cost_function_.Jacobian(x_list_[horizon_length_], u_zore_, 
                          lxx_list_[horizon_length_], luu_list_[horizon_length_]);
  luu_list_.back().setZero();
}

template <int dim_state, int dim_control>
void iLQR<dim_state, dim_control>::CalcCost() {

}

template <int dim_state, int dim_control>
double iLQR<dim_state, dim_control>::CalGradientNorm() {
  std::vector<double> vals(d_list_.size());
  Eigen::VectorXd v;
  for (int i = 0; i < vals.size(); ++i) {
    v = d_list_[i].cwiseAbs().array() / (u_list_[i].cwiseAbs().array() + 1);
    vals[i] = v.maxCoeff();
  }
  return std::accumulate(vals.begin(), vals.end(), 0.0) / vals.size();
}

template <int dim_state, int dim_control>
bool iLQR<dim_state, dim_control>::BackwardPass(
    const double reg) {
  p_list_[horizon_length_] = lx_list_[horizon_length_];
  P_list_[horizon_length_] = lxx_list_[horizon_length_];
  delta_V_[0] = 0.0; delta_V_[1] = 0.0;

  for (int k = horizon_length_ - 1; k >= 0; --k) {
    Qx_ = lx_list_[k] + (fx_list_[k].transpose() * p_list_[k + 1]);
    Qu_ = lu_list_[k] + (fu_list_[k].transpose() * p_list_[k + 1]);
    Qxx_ = lxx_list_[k] + (fx_list_[k].transpose() * P_list_[k + 1] * fx_list_[k]);
    Qux_ = lux_list_[k].transpose() + (fu_list_[k].transpose() * P_list_[k + 1] * fx_list_[k]);
    Quu_ = luu_list_[k] + (fu_list_[k].transpose() *  P_list_[k + 1] * fu_list_[k]);
    
    if (options_.solve_gain_method == SolveGainMethod::kCholeshy ||
        is_set_control_bound_ == false) {
      Qux_reg_ = Qux_;
      Qux_reg_.diagonal().array() += reg;
      // Cholesky 分解，判断 Qux_reg_ 是否正定
      Eigen::LLT<Eigen::Ref<Matrix>> Quu_fact(Qux_reg_);
      if (Quu_fact.info() != Eigen::Success) {
        return false;
      }
      K_list_[k] = Qux_;
      d_list_[k] = -Qu_;
      Quu_fact.solveInPlace(K_list_[k]);
      Quu_fact.solveInPlace(d_list_[k]);
    } else if (options_.solve_gain_method == SolveGainMethod::KBoxQp) {

    }

    delta_V_[0] += d_list_[k].transpose() * Qu_; 
    delta_V_[1] += 0.5 * d_list_[k].transpose() * Quu_ * d_list_[k];
    
    p_list_[k] = Qx_ + K_list_[k].transpose() * Quu_ * d_list_[k] + K_list_[k].transpose() * Qu_ + Qux_ * d_list_[k];
    P_list_[k] = Qxx_ + K_list_[k].transpose() * Quu_ * K_list_[k] + K_list_[k].transpose() * Qux_ + Qux_ * k_list_[k];
    P_list_[k] = 0.5 * (P_list_[k] + P_list_[k].transpose());
  }
  return true;
}

template <int dim_state, int dim_control>
void iLQR<dim_state, dim_control>::ForwardPass(
    const double alpha) {
  for (int k = 0; k < horizon_length_; ++k) {
    u_list[k] = u_old_list[k] + K_list_[k] * (x_list_[k] - x_old_list[k]) + alpha * d_list_[k];
    dynamics_.Evalute(x_list_[k], u_list[k], x_list[k + 1]);
  }
}

} // namespace ilqr