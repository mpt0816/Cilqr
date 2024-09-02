#include "algorithm/ilqr/corridor.h"

#include <iostream>

#include "algorithm/visualization/plot.h"

namespace planning {

Corridor::Corridor(const CorridorConfig& config, const Env& env) 
    : config_(config), env_(env) {}
  
void Corridor::Init(const CorridorConfig& config, const Env& env) {
  config_ = config;
  env_ = env;
}

bool Corridor::Plan(
    const DiscretizedTrajectory& trajectory, 
    CorridorConstraints* const corridor_constraints,
    ConvexPolygons* const convex_polygons,
    LaneConstraints* const left_lane_constraints,
    LaneConstraints* const right_lane_constraints) {

  if (trajectory.empty()) {
    ROS_ERROR("Corridor failed: Trajectory is empty!");
    return false;
  }

  if (corridor_constraints == nullptr ||
      convex_polygons == nullptr ||
      left_lane_constraints == nullptr ||
      right_lane_constraints == nullptr) {
    ROS_ERROR("Corridor failed: Input ptr is nullptr!");
    return false;
  }
  
  if (!BuildCorridorConstraints(
         trajectory, corridor_constraints, convex_polygons)) {
    ROS_ERROR("Corridor failed: Safe Corridors Build Failed!");
    return false;
  }

  if (!CalLeftLaneConstraints(left_lane_constraints)) {
    ROS_ERROR("Corridor failed: Left Lane Boundary Constraints Failed!");
    return false;
  }

  if (!CalRightLaneConstraints(right_lane_constraints)) {
    ROS_ERROR("Corridor failed: Right Lane Boundary Constraints Failed!");
    return false;
  }
  // CheckLaneConstraints(trajectory, *left_lane_constraints, *right_lane_constraints);
  return true;
}

bool Corridor::BuildCorridorConstraints(
    const DiscretizedTrajectory& trajectory, 
    CorridorConstraints* const corridor_constraints,
    ConvexPolygons* const convex_polygons) {
  points_for_corridors_.clear();
  corridor_constraints->clear();
  convex_polygons->clear();

  std::vector<math::Vec2d> obstacle_points;
  env_->QueryStaticObstaclesPoints(&obstacle_points, 
                                   config_.is_multiple_sample);
  int static_obs_pts_size = obstacle_points.size();
  Constraints constraints;
  ConvexPolygon polygon;

  for (const auto& pt : trajectory.trajectory()) {
    constraints.clear();
    polygon.clear();
    obstacle_points.resize(static_obs_pts_size);
    env_->QueryDynamicObstaclesPoints(pt.time, &obstacle_points, 
                                               config_.is_multiple_sample);
    AddCorridorPoints(pt, &obstacle_points);
    points_for_corridors_.push_back(obstacle_points);
    if(!BuildCorridor(pt.x, pt.y, obstacle_points, &constraints, &polygon)) {
      ROS_ERROR("Corridor failed: BuildCorridor Failed!");
      return false;
    }
    corridor_constraints->push_back(constraints);
    convex_polygons->push_back(polygon);
  }
  return true;
}

void Corridor::AddCorridorPoints(
      const TrajectoryPoint& pt, 
      std::vector<math::Vec2d>* const points) {
  if (points == nullptr) {
    return;
  }

  double cos_heading = std::cos(pt.theta);
  double sin_heading = std::sin(pt.theta);

  const double dx1 = cos_heading * config_.max_axis_x;
  const double dy1 = sin_heading * config_.max_axis_x;
  const double dx2 = sin_heading * config_.max_axis_y;
  const double dy2 = -cos_heading * config_.max_axis_y;
  
  std::vector<math::Vec2d> corners;
  corners.emplace_back(pt.x + dx1 + dx2, pt.y + dy1 + dy2);
  corners.emplace_back(pt.x + dx1 - dx2, pt.y + dy1 - dy2);
  corners.emplace_back(pt.x - dx1 - dx2, pt.y - dy1 - dy2);
  corners.emplace_back(pt.x - dx1 + dx2, pt.y - dy1 + dy2);

  double kSampleMultiple = config_.is_multiple_sample ? 5.0 : 1.0;
  double ratio_step = 1.0 / kSampleMultiple;
  for (int i = 0; i < corners.size(); ++i) {
    int index_next = (i + 1) % corners.size();
    for (double ratio = 0.0; ratio < 1.0 + math::kMathEpsilon; ratio += ratio_step) {
      double x = corners[i].x() * (1 - ratio) + corners[index_next].x() * ratio; 
      double y = corners[i].y() * (1 - ratio) + corners[index_next].y() * ratio; 
      points->emplace_back(x, y);
    }
  }
}
  
bool Corridor::BuildCorridor(
    const double origin_x, const double origin_y,  
    const std::vector<math::Vec2d>& points,  
    Constraints* const constraints,
    ConvexPolygon* const polygon) {
  if (points.size() == 0) {
    ROS_ERROR("Corridor failed: Points Size to Build Corridor is 0!!");
    return false;
  }

  if (constraints == nullptr || polygon == nullptr) {
    return false;
  }

  std::vector<math::Vec2d> filterd_points;
  for (size_t i = 0; i < points.size(); i++) {
    double dx = points[i].x() - origin_x;
    double dy = points[i].y() - origin_y;
    if (std::fabs(dx) > config_.max_diff_x || std::fabs(dy) > config_.max_diff_y) {
      continue;
    }

    double norm2 = std::sqrt(dx * dx + dy * dy);
    if (std::fabs(norm2) < math::kMathEpsilon) {
      continue;
    }  
    filterd_points.push_back(points[i]);
  }

  constraints->clear();
  polygon->clear();
  
  double safe_radius = config_.radius;

  // Data type must be cv::Point2f
  int sum = 0;
  std::vector<cv::Point2f> flipData(points.size() + 1, cv::Point2f(0, 0));
  for (size_t i = 0; i < filterd_points.size(); i++) {
    double dx = filterd_points[i].x() - origin_x;
    double dy = filterd_points[i].y() - origin_y;
    if (std::fabs(dx) > config_.max_diff_x || std::fabs(dy) > config_.max_diff_y) {
      continue;
    }

    double norm2 = std::sqrt(dx * dx + dy * dy);
    if (norm2 < config_.radius) {
      safe_radius = norm2;
    }
    if (std::fabs(norm2) < math::kMathEpsilon) {
      continue;
    }  
    flipData[i].x = dx + 2 * (config_.radius - norm2) * dx / norm2;
    flipData[i].y = dy + 2 * (config_.radius - norm2) * dy / norm2;
    ++sum;
  }

  if (sum < 4) {
    ROS_ERROR("Corridor failed: Flip Points Size to Build Corridor is less than 4!");
    return false;
  }
  
  std::vector<int> vertexIndice;
  cv::convexHull(flipData, vertexIndice, false, false);
    
  bool isOriginAVertex = false;
  int OriginIndex = -1;
  std::vector<cv::Point2f> vertexData;
  for (size_t i = 0; i < vertexIndice.size(); i++) {
    int v = vertexIndice[i];
    if (v == filterd_points.size()) {
      isOriginAVertex = true;
      OriginIndex = i;
      vertexData.push_back(cv::Point2f(origin_x, origin_y));
    }else {
      vertexData.push_back(cv::Point2f(filterd_points[v].x(), filterd_points[v].y()));
    }
  }
  
  double interior_x = 0.0;
  double interior_y = 0.0;
  if (isOriginAVertex) {
    int last_index = (OriginIndex - 1) % vertexIndice.size();
    int next_index = (OriginIndex + 1) % vertexIndice.size();
    double dx = (filterd_points[vertexIndice[last_index]].x() + origin_x + 
                 filterd_points[vertexIndice[next_index]].x()) / 3 - origin_x;
    double dy = (filterd_points[vertexIndice[last_index]].y() + origin_y + 
                 filterd_points[vertexIndice[next_index]].y()) / 3 - origin_y;
    double d = std::sqrt(dx * dx + dy * dy);
    interior_x = 0.99 * safe_radius * dx / d + origin_x;
    interior_y = 0.99 * safe_radius * dy / d + origin_y;
  } else {
    interior_x = origin_x;
    interior_y = origin_y;
  }

  std::vector<int> vIndex2;
  cv::convexHull(vertexData, vIndex2, false, false); // counterclockwise right-hand

  std::vector<Eigen::Vector3f> temp_constraints; // (a,b,c) a x + b y <= c
  for (size_t j = 0; j < vIndex2.size(); j++) {
    int jplus1 = (j + 1) % vIndex2.size();
    cv::Point2f rayV = vertexData[vIndex2[jplus1]] - vertexData[vIndex2[j]];
    Eigen::Vector2f normalJ(rayV.y, -rayV.x);  // point to outside
    normalJ.normalize();
    int indexJ = vIndex2[j];
    while (indexJ != vIndex2[jplus1]) {
      double c = (vertexData[indexJ].x - interior_x) * normalJ(0) + 
                 (vertexData[indexJ].y - interior_y) * normalJ(1);
      temp_constraints.push_back(Eigen::Vector3f(normalJ(0), normalJ(1), c));
      indexJ = (indexJ + 1) % vertexData.size();
    }
  }    

  std::vector<cv::Point2f> dualPoints(temp_constraints.size(), cv::Point2f(0,0));
  for (size_t i = 0; i < temp_constraints.size(); i++) {
    dualPoints[i].x = temp_constraints[i](0) / temp_constraints[i](2);
    dualPoints[i].y = temp_constraints[i](1) / temp_constraints[i](2);
  }
    
  std::vector<cv::Point2f> dualVertex;
  cv::convexHull(dualPoints, dualVertex, true, false);

  for (size_t i = 0; i < dualVertex.size(); i++) {
    int iplus1 = (i + 1) % dualVertex.size();
    cv::Point2f rayi = dualVertex[iplus1] - dualVertex[i];
    double c = rayi.y * dualVertex[i].x - rayi.x * dualVertex[i].y;
    polygon->push_back(Eigen::Vector2d(interior_x + rayi.y / c, interior_y - rayi.x / c));
  }

  for (size_t i = 0; i < polygon->size(); i++) {
    int iplus1 = (i + 1) % polygon->size();
    Eigen::Vector2d rayi = polygon->at(iplus1) - polygon->at(i);
    double c = -rayi[1] * polygon->at(i)[0] + rayi[0] * polygon->at(i)[1];
    // if (-origin_x * rayi[1] + origin_y * rayi[0] <= c) {
    //   std::cout << "corridor constraint correct!" << std::endl;
    // } else {
    //   std::cout << "corridor constraint NOT correct!" << std::endl;
    // }
    constraints->push_back(Eigen::Vector3d(-rayi[1], rayi[0], c));
  }
  return true;
}
  
bool Corridor::CalLeftLaneConstraints(
    LaneConstraints* const left_lane_constraints) {
  if (left_lane_constraints == nullptr) {
    return false;
  }
  left_lane_constraints->clear();

  std::vector<math::Vec2d> lane_boundary = 
      LaneBoundarySample(env_->left_road_barrier());
  if (lane_boundary.size() < 2) {
    return false;
  }

  for (int i = 1; i < lane_boundary.size(); ++i) {
    math::LineSegment2d segment(lane_boundary[i], lane_boundary[i - 1]);
    Eigen::Vector3d cons = HalfPlaneConstraint(lane_boundary[i], lane_boundary[i - 1]);
    left_lane_constraints->push_back(std::make_pair(cons, segment));
  }
  return true;
}

bool Corridor::CalRightLaneConstraints(
    LaneConstraints* const right_lane_constraints) {
  if (right_lane_constraints == nullptr) {
    return false;
  }
  right_lane_constraints->clear();

  std::vector<math::Vec2d> lane_boundary = 
      LaneBoundarySample(env_->right_road_barrier());
  if (lane_boundary.size() < 2) {
    return false;
  }

  for (int i = 1; i < lane_boundary.size(); ++i) {
    math::LineSegment2d segment(lane_boundary[i - 1], lane_boundary[i]);
    Eigen::Vector3d cons = HalfPlaneConstraint(lane_boundary[i - 1], lane_boundary[i]);
    right_lane_constraints->push_back(std::make_pair(cons, segment));
  }
  return true;
}

std::vector<math::Vec2d> Corridor::LaneBoundarySample(
    const std::vector<math::Vec2d>& lane_boundary) {
  std::vector<math::Vec2d> sampled_lane_boundary;
  math::Vec2d last_pt = lane_boundary.front();
  sampled_lane_boundary.push_back(last_pt);
  for (const auto& pt : lane_boundary) {
    if (std::hypot(pt.x() - last_pt.x(), pt.y() - last_pt.y()) 
          >= config_.lane_segment_length - math::kMathEpsilon) {
      sampled_lane_boundary.push_back(pt);
      last_pt = pt;
    }
  }
  return sampled_lane_boundary;
}

Eigen::Vector3d Corridor::HalfPlaneConstraint(
    const math::Vec2d& start_pt,
    const math::Vec2d& end_pt) {
  math::Vec2d n = end_pt - start_pt;
  double a = n.y();
  double b = -n.x();
  double c = a * start_pt.x() + b * start_pt.y();
  Eigen::Vector3d res(a, b, c);
  return res;
}

void Corridor::CheckLaneConstraints(
    const DiscretizedTrajectory& trajectory,
    const LaneConstraints& left_lane_constraints,
    const LaneConstraints& right_lane_constraints) {
  std::pair<Eigen::Vector3d, math::LineSegment2d> cons;
  for (const auto& pt : trajectory.trajectory()) {
    cons = FindNeastLaneSegment(pt.x, pt.y, left_lane_constraints);
    if (pt.x * cons.first[0] + pt.y * cons.first[1] <= cons.first[2])  {
      std::cout << "Left Lane constraint correct!" << std::endl;
      // visualization::PlotPoint(
      //       math::Vec2d(pt.x, pt.y), 0.3, visualization::Color::Cyan, 1, "Vehicle Position");
      // visualization::PlotLineSegment(
      //       cons.second, 0.3, visualization::Color::Cyan, 1, "Error lane seg");
      // visualization::Trigger();
      // ros::Duration(0.3).sleep();
    } else {
      visualization::PlotPoint(
            math::Vec2d(pt.x, pt.y), 0.3, visualization::Color::Cyan, 1, "Vehicle Position");
      visualization::PlotLineSegment(
            cons.second, 0.3, visualization::Color::Cyan, 1, "Error lane seg");
      visualization::Trigger();
      
      std::cout << "Left Lane constraint NOT correct!" << std::endl;
      std::cout << "x, y: " << pt.x << ", " << pt.y 
                << ", plane, start point: " << cons.second.start().x() << ", " << cons.second.start().y() 
                << ", end point: " << cons.second.end().x() << ", " << cons.second.end().y() 
                << ", plane: " << cons.first[0] << ", " << cons.first[1] << ", " << cons.first[2] 
                << ", ax + by = " << pt.x * cons.first[0] + pt.y * cons.first[1] << std::endl;

      ros::Duration(0.3).sleep();
    }

    cons = FindNeastLaneSegment(pt.x, pt.y, right_lane_constraints);
    if (pt.x * cons.first[0] + pt.y * cons.first[1] <= cons.first[2])  {
      // visualization::PlotPoint(
      //       math::Vec2d(pt.x, pt.y), 0.3, visualization::Color::Cyan, 1, "Vehicle Position");
      // visualization::PlotLineSegment(
      //       cons.second, 0.3, visualization::Color::Cyan, 1, "Error lane seg");
      // visualization::Trigger();
      // ros::Duration(0.3).sleep();
      std::cout << "Right lane constraint correct!" << std::endl;
    } else {
      visualization::PlotPoint(
            math::Vec2d(pt.x, pt.y), 0.3, visualization::Color::Cyan, 1, "Vehicle Position");
      visualization::PlotLineSegment(
            cons.second, 0.3, visualization::Color::Cyan, 1, "Error lane seg");
      visualization::Trigger();
      ros::Duration(0.3).sleep();
      std::cout << "x, y: " << pt.x << ", " << pt.y 
                << ", plane, start point: " << cons.second.start().x() << ", " << cons.second.start().y()
                << ", end point: " << cons.second.end().x() << ", " << cons.second.end().y() 
                << ", plane: " << cons.first[0] << ", " << cons.first[1] << ", " << cons.first[2] 
                << ", ax + by = " << pt.x * cons.first[0] + pt.y * cons.first[1] << std::endl;
      std::cout << "Right lane  constraint NOT correct!" << std::endl;
    }
  }
}

std::pair<Eigen::Vector3d, math::LineSegment2d> Corridor::FindNeastLaneSegment(
    const double x, const double y, 
    const std::vector<std::pair<Eigen::Vector3d, math::LineSegment2d>>& lane_segs) {
  double min_dis = std::numeric_limits<double>::max();
  int min_index = -1;
  for (int i = 0; i < lane_segs.size(); ++i) {
    double dis = lane_segs[i].second.DistanceTo(math::Vec2d(x, y));
    if (dis < min_dis) {
      min_dis = dis;
      min_index = i;
    }
  }
  std::cout << "min dis: " << min_dis << std::endl;
  return lane_segs[min_index];
}

} // namespace planning