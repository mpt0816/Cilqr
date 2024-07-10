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

#include "trajectory_planner.h"

#include "visualization/plot.h"
#include "timer.h"

#include <iostream>

namespace planning {

bool TrajectoryPlanner::Plan(const StartState& state, DiscretizedTrajectory& result) {
  DiscretizedTrajectory coarse_trajectory;
  utils::time dp_start_time = utils::Time();
  if(!dp_.Plan(state.x, state.y, state.theta, coarse_trajectory)) {
    ROS_ERROR("DP failed");
    return false;
  }
  utils::time dp_end_time = utils::Time();
  double dp_time_cost = utils::Duration(dp_start_time, dp_end_time);
  std::cout << "DP time cost: " << dp_time_cost << std::endl;
   
  CorridorConstraints corridor_constraints;
  ConvexPolygons convex_polygons;
  LeftLaneConstraints left_lane_constraints;
  RightLaneConstraints right_lane_constraints;
  
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

  visualization::Plot(coarse_x, coarse_y, 0.1, visualization::Color::Cyan, 1, "Coarse Trajectory");
  visualization::PlotPoints(coarse_x, coarse_y, 0.3, visualization::Color::Cyan, 2, "Coarse Trajectory");
  visualization::Trigger();

  std::vector<double> opti_x, opti_y, opti_v;
  Trajectory result_data;
  double incremental_s = 0.0;
  int nfe = config_.tf / config_.delta_t + 1;
  for(int i = 0; i < nfe; i++) {
    TrajectoryPoint tp;
    incremental_s += i > 0 ? hypot(coarse_trajectory.trajectory()[i].x - coarse_trajectory.trajectory()[i - 1].x, coarse_trajectory.trajectory()[i].y - coarse_trajectory.trajectory()[i - 1].y) : 0.0;
    tp.s = incremental_s;

    tp.x = coarse_trajectory.trajectory()[i].x;
    tp.y = coarse_trajectory.trajectory()[i].y;
    tp.theta = coarse_trajectory.trajectory()[i].theta;
    tp.velocity = coarse_trajectory.trajectory()[i].velocity;
    tp.kappa = tan(coarse_trajectory.trajectory()[i].theta) / config_.vehicle.wheel_base;

    opti_x.push_back(tp.x);
    opti_y.push_back(tp.y);
    opti_v.push_back(tp.velocity);

    result_data.push_back(tp);
  }

  visualization::PlotTrajectory(opti_x, opti_y, opti_v, config_.vehicle.max_velocity, 0.1, visualization::Color::Green, 1, "Optimized Trajectory");
  visualization::Trigger();

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