/***********************************************************************************
 *  C++ Source Codes for "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method".
 ***********************************************************************************
 *  Copyright (C) 2022 Bai Li
 *  Users are suggested to cite the following article when they use the source codes.
 *  Bai Li et al., "Autonomous Driving on Curvy Roads without Reliance on
 *  Frenet Frame: A Cartesian-based Trajectory Planning Method",
 *  IEEE Transactions on Intelligent Transportation Systems, 2022.
 ***********************************************************************************/

#pragma once

#include "algorithm/params/vehicle_param.h"

namespace planning {

struct LateralTrackerConfig {
  double weight_l = 1e-1;
  double weight_theta = 1e-12;
  double weight_delta = 1e-12;
  double weight_delta_rate = 0.1;

  double preview_time = 0.2;
};

struct LongitudinalTrackerConfig {
  double weight_s = 5.0 * 1e-1;
  double weight_v = 1e-12;
  double weight_a = 1e-12;
  double weight_j = 0.1;

  double preview_time = 0.0;
};

struct TrackerConfig {
  double sumulation_dt = 0.01;
  double dt = 0.1;
  double tolerance = 0.01;
  uint max_num_iteration = 150;
  LateralTrackerConfig lateral_config;
  LongitudinalTrackerConfig longitudinal_config;
};

struct Weights {
  double jerk = 1;
  double delta_rate = 1;

  double x_target = 0.5;
  double y_target = 0.5;
  double theta = 1e-3;
  double v = 0.0;
  double a = 0.0;
  double delta = 0.0;
};

struct IlqrConfig {
  int num_of_disc = 5;
  double safe_margin = 0.2;
  double t = 100.0;
  double t_rate = 10.0;
  Weights weights;
  int max_iter_num = 200;

  double abs_cost_tol = 1e-2;
  double rel_cost_tol = 1e-2;

  double alpha = 1.0;
  double gamma = 0.5;
  double rho = 1e-9;

  TrackerConfig tracker_config;
};

struct CorridorConfig {
  bool is_multiple_sample = false;
  double max_diff_x = 25.0;
  double max_diff_y = 25.0;
  double radius = 150.0;

  double max_axis_x = 10.0;
  double max_axis_y = 10.0;
  
  // 
  double lane_segment_length = 5.0;
};

struct PlannerConfig {
  /**
   * Number of finite elements used to discretize an OCP
   */
  int nfe = 320;

  double delta_t = 0.1;

  /**
   * Time horizon length (s)
   */
  double tf = 8;

  /**
   * nominal velocity
   */
  double dp_nominal_velocity = 10.0;

  /**
   * cost of obstacles, should be set larger to avoid collision with obstacles
   */
  double dp_w_obstacle = 1000;

  /**
   * lateral cost, the larger the trajectory the closer to the reference line
   */
  double dp_w_lateral = 0.1;

  /**
   * lateral change cost, dl/ds, penalty for lateral change
   */
  double dp_w_lateral_change = 0.5;

  /**
   * lateral change cost, dl/dt, penalty for sudden lateral change
   */
  double dp_w_lateral_velocity_change = 1.0;

  /**
   * longitudinal velocity cost, velocity to the nominal velocity
   */
  double dp_w_longitudinal_velocity_bias = 10.0;

  /**
   * Cost of longitudinal velocity change, ds/dt
   */
  double dp_w_longitudinal_velocity_change = 1.0;

  /**
   * maximum iteration count for corridor expansion
   */
  int corridor_max_iter = 1000;

  /**
   * increment limit for corridor expansion
   */
  double corridor_incremental_limit = 20.0;

  /**
   * Weighting parameter in Eq.(2)
   */
  double opti_w_u = 0.5;

  /**
   * weighting parameter in Eq.(3)
   */
  double opti_w_r_theta = 2.0;

  /**
   * weighting parameter in Eq.(4)
   */
  double opti_w_rw = 5.0;

  /**
   * Maximum iteration number in Alg.1
   */
  int opti_iter_max = 5;

  /**
   * Initial value of weighting parameter w_penalty
   */
  double opti_w_penalty0 = 1e5;

  /**
   * Multiplier to enlarge w_penalty during the iterations
   */
  double opti_alpha = 10;

  /**
   * Violation tolerance w.r.t. the softened nonlinear constraints
   */
  double opti_varepsilon_tol = 1e-4;

  VehicleParam vehicle;

  CorridorConfig corridor_config;

  IlqrConfig ilqr_config;

  TrackerConfig tracker_config;
};

} // namespace planning