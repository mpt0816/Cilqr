#pragma once

#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <random>
#include <vector>

#include <Eigen/Eigen>

#include "algorithm/params/planner_config.h"
#include "algorithm/utils/discretized_trajectory.h"
#include "algorithm/utils/environment.h"
#include "algorithm/math/line_segment2d.h"

namespace planning {

using ConvexPolygon = std::vector<Eigen::Vector2d>;
using ConvexPolygons = std::vector<ConvexPolygon>;
// Note: ax + by < c, Vector3d << a, b, c
using Constraints = std::vector<Eigen::Vector3d>;
using CorridorConstraints = std::vector<Constraints>;

using LaneConstraints = 
    std::vector<std::pair<Eigen::Vector3d, math::LineSegment2d>>;

class Corridor {
 public:
  Corridor(const CorridorConfig& config, const Env& env);
  
  void Init(const CorridorConfig& config, const Env& env);

  bool Plan(
      const DiscretizedTrajectory& trajectory, 
      CorridorConstraints* const corridor_constraints,
      ConvexPolygons* const convex_polygons,
      LaneConstraints* const left_lane_constraints,
      LaneConstraints* const right_lane_constraints);
  
  std::vector<std::vector<math::Vec2d>> points_for_corridors() {
    return points_for_corridors_;
  }

 private:
  bool BuildCorridorConstraints(
      const DiscretizedTrajectory& trajectory, 
      CorridorConstraints* const corridor_constraints,
      ConvexPolygons* const convex_polygons);
  
  bool BuildCorridor(
      const double origin_x, const double origin_y,  
      const std::vector<math::Vec2d>& points,  
      Constraints* const constraints,
      ConvexPolygon* const polygon);
  
  bool CalLeftLaneConstraints(
      LaneConstraints* const left_lane_constraints);

  bool CalRightLaneConstraints(
      LaneConstraints* const right_lane_constraints);

  bool CalConstraintFromPoints(
      const math::Vec2d& start_pt,
      const math::Vec2d& end_pt,
      Eigen::Vector3d* const constraint);
  
  void AddCorridorPoints(
      const TrajectoryPoint& pt, 
      std::vector<math::Vec2d>* const points);

  std::vector<math::Vec2d> LaneBoundarySample(
      const std::vector<math::Vec2d>& lane_boundary);

  Eigen::Vector3d HalfPlaneConstraint(
      const math::Vec2d& start_pt,
      const math::Vec2d& end_pt);

  void CheckLaneConstraints(
      const DiscretizedTrajectory& trajectory,
      const LaneConstraints& left_lane_constraints,
      const LaneConstraints& right_lane_constraints);

  std::pair<Eigen::Vector3d, math::LineSegment2d> 
  FindNeastLaneSegment(
    const double x, const double y, 
    const std::vector<std::pair<Eigen::Vector3d, math::LineSegment2d>>& lane_segs);
 
 private:
  CorridorConfig config_;
  Env env_;
  std::vector<std::vector<math::Vec2d>> points_for_corridors_;
};

} // namespace planning