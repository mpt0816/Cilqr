#include <ros/ros.h>

#include "geometry_msgs/PoseStamped.h"
#include "planning/CenterLine.h"
#include "planning/Obstacles.h"
#include "planning/DynamicObstacles.h"

#include "algorithm/planner/trajectory_planner.h"
#include "algorithm/visualization/plot.h"

namespace planning {

class PlanningNode {
 public:
  explicit PlanningNode(const ros::NodeHandle& nh);

 private:
  void CenterLineCallback(const CenterLineConstPtr& msg);

  void ObstaclesCallback(const ObstaclesConstPtr& msg);

  void DynamicObstaclesCallback(const DynamicObstaclesConstPtr& msg); 

  void PlanCallback(const geometry_msgs::PoseStampedConstPtr& msg);

 private:
  ros::NodeHandle nh_;
  PlannerConfig config_;
  Env env_;
  std::shared_ptr<TrajectoryPlanner> planner_;
  StartState state_;

  ros::Subscriber center_line_subscriber_; 
  ros::Subscriber obstacles_subscriber_; 
  ros::Subscriber dynamic_obstacles_subscriber_; 
  ros::Subscriber goal_subscriber_;

  void PlotVehicle(const int id, const math::Pose& pt, const double phi);

  std::array<math::Box2d, 4> GenerateTireBoxes(
      const math::Pose& pose, const double phi = 0.0) const;
};

} // namsepace planning