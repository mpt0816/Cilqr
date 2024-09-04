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

#include "algorithm/planner/trajectory_planner.h"

#include <iostream>

#include "algorithm/visualization/plot.h"
#include "algorithm/utils/timer.h"
#include "algorithm/visualization/figure_plot.h"

namespace planning {

TrajectoryPlanner::TrajectoryPlanner(const PlannerConfig& config, const Env& env)
    : config_(config)
    , dp_(config, env)
    , corridor_(config.corridor_config, env)
    , ilqr_optimizer_(config.ilqr_config, config.vehicle, config.tf, config.delta_t) {}

bool TrajectoryPlanner::Plan(
    const StartState& state, DiscretizedTrajectory& result) {
  DiscretizedTrajectory coarse_trajectory;
  utils::time dp_start_time = utils::Time();
  if(!dp_.Plan(state.x, state.y, state.theta, coarse_trajectory)) {
    ROS_ERROR("DP failed");
    return false;
  }
  utils::time dp_end_time = utils::Time();
  double dp_time_cost = utils::Duration(dp_start_time, dp_end_time);
  std::cout << "DP time cost: " << dp_time_cost << std::endl;

  visualization::FigurePlot figure_plot;
  // figure_plot.Plot(coarse_trajectory);
   
  CorridorConstraints corridor_constraints;
  ConvexPolygons convex_polygons;
  LaneConstraints left_lane_constraints;
  LaneConstraints right_lane_constraints;
  
  utils::time corridor_start_time = utils::Time();
  if (!corridor_.Plan(coarse_trajectory, 
                      &corridor_constraints,
                      &convex_polygons, 
                      &left_lane_constraints, 
                      &right_lane_constraints)) {
    ROS_ERROR("Corridor failed");
    return false;
  }
  utils::time corridor_end_time = utils::Time();
  double corridor_time_cost = utils::Duration(corridor_start_time, corridor_end_time);
  std::cout << "Corridor time cost: " << corridor_time_cost << std::endl;

  // visualization::PlotConvexPolygons(convex_polygons, 0.1, visualization::Color::Cyan, 1, "Safe Corridors");
  // visualization::Trigger();

  std::vector<double> coarse_x, coarse_y;
  for(auto &pt: coarse_trajectory.trajectory()) {
    coarse_x.push_back(pt.x); coarse_y.push_back(pt.y);
  }

  visualization::Plot(coarse_x, coarse_y, 0.1, visualization::Color::Red, 1, "Coarse Trajectory");
  // visualization::PlotPoints(coarse_x, coarse_y, 0.3, visualization::Color::Cyan, 2, "Coarse Trajectory");
  visualization::Trigger();
  
  TrajectoryPoint start_state;
  start_state.x = state.x; start_state.y = state.y;
  start_state.velocity = state.v; start_state.theta = state.theta;
  DiscretizedTrajectory opt_trajectory;
  std::vector<DiscretizedTrajectory> iter_trajs;

  utils::time ilqr_start_time = utils::Time();
  bool status = ilqr_optimizer_.Plan(start_state,
                                     coarse_trajectory, 
                                     corridor_constraints,
                                     left_lane_constraints,
                                     right_lane_constraints,
                                     &opt_trajectory,
                                     &iter_trajs);
  utils::time ilqr_end_time = utils::Time();
  double ilqr_time_cost = utils::Duration(ilqr_start_time, ilqr_end_time);
  std::cout << "ilqr time cost: " << ilqr_time_cost << std::endl;

  if (opt_trajectory.empty()) {
    ROS_ERROR("ilqr failed");
    return false;
  }

  // figure_plot.Plot(coarse_trajectory, opt_trajectory);
  figure_plot.Plot(coarse_trajectory, iter_trajs, ilqr_optimizer_.cost());

  // opt_trajectory = coarse_trajectory;
  std::vector<double> opti_x, opti_y, opti_v;
  Trajectory result_data;
  double incremental_s = 0.0;
  int nfe = config_.tf / config_.delta_t + 1;
  for(int i = 0; i < nfe; i++) {
    TrajectoryPoint tp;
    tp.time = config_.delta_t * i;
    incremental_s += i > 0 ? hypot(opt_trajectory.trajectory()[i].x - opt_trajectory.trajectory()[i - 1].x, 
                                   opt_trajectory.trajectory()[i].y - opt_trajectory.trajectory()[i - 1].y) : 0.0;
    tp.s = incremental_s;
    tp.x = opt_trajectory.trajectory()[i].x;
    tp.y = opt_trajectory.trajectory()[i].y;
    tp.theta = opt_trajectory.trajectory()[i].theta;
    tp.velocity = opt_trajectory.trajectory()[i].velocity;
    tp.kappa = std::tan(opt_trajectory.trajectory()[i].delta) / config_.vehicle.wheel_base;
    tp.a = opt_trajectory.trajectory()[i].a;
    tp.jerk = opt_trajectory.trajectory()[i].jerk;
    tp.delta = opt_trajectory.trajectory()[i].delta;
    tp.delta_rate = opt_trajectory.trajectory()[i].delta_rate;

    opti_x.push_back(tp.x);
    opti_y.push_back(tp.y);
    opti_v.push_back(tp.velocity);

    result_data.push_back(tp);
  }

  visualization::PlotTrajectory(opti_x, opti_y, opti_v, config_.vehicle.max_velocity, 0.1, visualization::Color::Green, 1, "Optimized Trajectory");
  visualization::Trigger();

  // for (int i = 0; i < iter_trajs.size(); ++i) {
    std::vector<double> tem_opti_x, tem_opti_y, tem_opti_v;
    for(int j = 0; j < nfe; j++) {
      tem_opti_x.push_back(iter_trajs[0].trajectory()[j].x);
      tem_opti_y.push_back(iter_trajs[0].trajectory()[j].y);
      tem_opti_v.push_back(iter_trajs[0].trajectory()[j].velocity);
    }
    visualization::PlotTrajectory(tem_opti_x, tem_opti_y, tem_opti_v, config_.vehicle.max_velocity, 0.1, visualization::Color::Yellow, 1, "guess Optimized Trajectory");
    visualization::Trigger();
  //   ros::Duration(0.1 * 5).sleep();
  //   break;
  // }
  

  result = DiscretizedTrajectory(result_data);
  convex_polygons_ = convex_polygons;

  left_lane_boundary_.clear();
  for (const auto& seg : left_lane_constraints) {
    left_lane_boundary_.push_back(seg.second);
  }
  
  right_lane_boundary_.clear();
  for (const auto& seg : right_lane_constraints) {
    right_lane_boundary_.push_back(seg.second);
  }

  visualization::PlotLineSegments(left_lane_boundary_, 0.1, visualization::Color::Cyan, 1, "Left Lane Segments");
  visualization::PlotLineSegments(right_lane_boundary_, 0.1, visualization::Color::Cyan, 1, "Right Lane Segments");
  visualization::Trigger();

  return true;
}
} // namespace planning