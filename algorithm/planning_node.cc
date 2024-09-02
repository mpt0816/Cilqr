#include "algorithm/planning_node.h"

#include <iostream>

namespace planning {

PlanningNode::PlanningNode(const ros::NodeHandle& nh) : nh_(nh) {
  env_ = std::make_shared<Environment>(config_);
  planner_ = std::make_shared<TrajectoryPlanner>(config_, env_);

  center_line_subscriber_ = nh_.subscribe(
      "/center_line", 1, &PlanningNode::CenterLineCallback, this);
  obstacles_subscriber_ = nh_.subscribe(
      "/obstacles", 1, &PlanningNode::ObstaclesCallback, this);
  dynamic_obstacles_subscriber_ = nh_.subscribe(
      "/dynamic_obstacles", 1,
      &PlanningNode::DynamicObstaclesCallback, this);

  goal_subscriber_ = nh_.subscribe(
      "/move_base_simple/goal", 1, &PlanningNode::PlanCallback, this);

  state_.x = 0.0;
  state_.y = 0.0;
  state_.theta = 0.0;
  state_.v = 10.0;
  state_.phi = 0.0;
  state_.a = 0.0;
  state_.omega = 0.0;
}

void PlanningNode::CenterLineCallback(const CenterLineConstPtr& msg) {
  Trajectory data;
  for (auto &pt: msg->points) {
    TrajectoryPoint tp;
    tp.s = pt.s;
    tp.x = pt.x;
    tp.y = pt.y;
    tp.theta = pt.theta;
    tp.kappa = pt.kappa;
    tp.left_bound = pt.left_bound;
    tp.right_bound = pt.right_bound;
    data.push_back(tp);
  }

  env_->set_reference(DiscretizedTrajectory(data));
  env_->Visualize();
}

void PlanningNode::ObstaclesCallback(const ObstaclesConstPtr& msg) {
  env_->obstacles().clear();
  for (auto &obstacle: msg->obstacles) {
    std::vector<math::Vec2d> points;
    for (auto &pt: obstacle.points) {
      points.emplace_back(pt.x, pt.y);
    }
    env_->obstacles().emplace_back(points);
  }
  env_->Visualize();
}

void PlanningNode::DynamicObstaclesCallback(const DynamicObstaclesConstPtr& msg) {
  env_->dynamic_obstacles().clear();
  for (auto& obstacle: msg->obstacles) {
    Environment::DynamicObstacle dynamic_obstacle;

    for (auto& tp: obstacle.trajectory) {
      math::Pose coord(tp.x, tp.y, tp.theta);
      std::vector<math::Vec2d> points;
      for (auto& pt: obstacle.polygon.points) {
        points.push_back(coord.transform({pt.x, pt.y, 0.0}));
      }
      math::Polygon2d polygon(points);
      dynamic_obstacle.emplace_back(tp.time, points);
    }
    env_->dynamic_obstacles().push_back(dynamic_obstacle);
  }
  env_->Visualize();
}

void PlanningNode::PlanCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  DiscretizedTrajectory result;
  double dt = config_.delta_t;
  int nfe = config_.tf / config_.delta_t + 1;
  if (planner_->Plan(state_, result)) {
    auto convex_polygons = planner_->SafeCorridors();
    auto points_for_corridors = planner_->points_for_corridors();
    for (int i = 0; i < nfe; i++) {
      double time = dt * i;
      auto dynamic_obstacles = env_->QueryDynamicObstacles(time);
      for (auto& obstacle: dynamic_obstacles) {
        int hue = int((double) obstacle.first / env_->dynamic_obstacles().size() * 320);

        visualization::PlotPolygon(
            obstacle.second, 0.2, visualization::Color::fromHSV(hue, 1.0, 1.0), 
            obstacle.first, "Online Obstacle");
      }
      visualization::PlotConvexPolygon(
          convex_polygons[i], 0.1, visualization::Color::Cyan, 1, "Safe Corridors");

      visualization::PlotPoints(
          points_for_corridors[i], 0.3, visualization::Color::Cyan, 1, "Corridor Points");
          
      auto &pt = result.trajectory().at(i);
      PlotVehicle(1, {pt.x, pt.y, pt.theta}, atan(pt.kappa * config_.vehicle.wheel_base));
      ros::Duration(dt * 1.5).sleep();
    }

    visualization::Trigger();
  }
}

void PlanningNode::PlotVehicle(const int id, const math::Pose& pt, const double phi) {
  auto tires = GenerateTireBoxes({pt.x(), pt.y(), pt.theta()}, phi);

  int tire_id = 1;
  for (auto &tire: tires) {
    visualization::PlotPolygon(math::Polygon2d(tire), 0.1, visualization::Color::White, id * (tire_id++),
                               "Tires");
  }
  visualization::PlotPolygon(math::Polygon2d(config_.vehicle.GenerateBox({pt.x(), pt.y(), pt.theta()})), 0.2,
                             visualization::Color::Yellow, id, "Footprint");
  visualization::Trigger();
}

std::array<math::Box2d, 4> PlanningNode::GenerateTireBoxes(
    const math::Pose& pose, const double phi) const {
  auto front_pose = pose.extend(config_.vehicle.wheel_base);
  auto track_width = config_.vehicle.width - 0.195;
  double rear_track_width_2 = track_width / 2, front_track_width_2 = track_width / 2;
  double box_length = 0.6345;
  double sin_t = sin(pose.theta());
  double cos_t = cos(pose.theta());
  return {
    math::Box2d({pose.x() - rear_track_width_2 * sin_t, pose.y() + rear_track_width_2 * cos_t}, pose.theta(),
                box_length, 0.195),
    math::Box2d({pose.x() + rear_track_width_2 * sin_t, pose.y() - rear_track_width_2 * cos_t}, pose.theta(),
                box_length, 0.195),
    math::Box2d({front_pose.x() - front_track_width_2 * sin_t, front_pose.y() + front_track_width_2 * cos_t},
                front_pose.theta() + phi, box_length, 0.195),
    math::Box2d({front_pose.x() + front_track_width_2 * sin_t, front_pose.y() - front_track_width_2 * cos_t},
                front_pose.theta() + phi, box_length, 0.195),
  };
}

} // namespace planning