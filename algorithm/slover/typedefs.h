#pragma once

#include <vector>
#include <Eigen/Eigen>


namespace ilqr {

using Vector = Eigen::Vector<double, Eigen::Dynamic>;
using Matrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXdRef = Eigen::Ref<const Eigen::VetorXd>;

typedef std::vector<Eigen::VectorXd, Eigen::aligned_allocator<VectorXd>> VectorXdList;
typedef std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<MatrixXd>> MatrixXdList;

enum class ErrorCode {
  kNoError,
  kStateDimUnknown,
  kControlDimUnknown,
};

enum class SolveiLQRMethod {
  kBarrier;
  kAL;
};

enum class SolveGainMethod {
  kCholeshy,
  KBoxQp,
};

enum class SolverStatus {
  kSuccess,
  kUnsolved,
  kMaxIterations,
  kMaxObjectiveExceeded,
  kStateOutOfBounds,
  kInputOutOfBounds,
  kMeritFunGradientTooSmall,
};

struct Options {
  Options() = default;

  SolveiLQRMethod solve_ilqr_method = SolveiLQRMethod::kBarrier;
  
  int iter_max = 200;
  double tol_abs = 1e-3;
  double tol_rel = 1e-3;
  double regularization_ratio = 1.6; // 1.5 ~ 2.0
  double regularization_min = 1e-8;
  double regularization_max = 1e11;
  double gradient_norm_min = 1e-6;
  double beta_min = 1e-4;
  double beta_max = 10.0;
  SolveGainMethod solve_gain_method = SolveGainMethod::kCholeshy;
};


} // namespace ilqr