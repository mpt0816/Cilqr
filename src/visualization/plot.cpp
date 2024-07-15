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

#include "visualization/plot.h"

namespace planning {
namespace visualization {
namespace {
std::string frame_ = "map";
std::mutex mutex_;

ros::Publisher publisher_;
visualization_msgs::MarkerArray arr_;
}

void Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic) {
  frame_ = frame;
  publisher_ = node.advertise<visualization_msgs::MarkerArray>(topic, 10, true);
}

void
Plot(const Vector &xs, const Vector &ys, double width, Color color, int id, const std::string &ns) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    pt.z = 0.1 * id;
    msg.points.push_back(pt);
  }

  mutex_.lock();
  arr_.markers.push_back(msg);
  mutex_.unlock();
}

void Plot(const Vector &xs, const Vector &ys, double width,
          const std::vector<Color> &color, int id, const std::string &ns) {
  assert(xs.size() == color.size());

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width;

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
    msg.colors.push_back(color[i].toColorRGBA());
  }

  mutex_.lock();
  arr_.markers.push_back(msg);
  mutex_.unlock();
}


void PlotPolygon(const Vector &xs, const Vector &ys, double width, Color color, int id,
                 const std::string &ns) {
  auto xxs = xs;
  auto yys = ys;
  xxs.push_back(xxs[0]);
  yys.push_back(yys[0]);
  Plot(xxs, yys, width, color, id, ns);
}

void PlotPolygon(const Polygon2d &polygon, double width, Color color, int id,
                 const std::string &ns) {
  std::vector<double> xs, ys;
  for (auto &pt: polygon.points()) {
    xs.push_back(pt.x());
    ys.push_back(pt.y());
  }
  PlotPolygon(xs, ys, width, color, id, ns);
}

void PlotConvexPolygon(
    const std::vector<Eigen::Vector2d>& polygon, 
    double width,
    Color color,
    int id, const std::string &ns) {
  std::vector<double> xs, ys;
  for (const auto& pt: polygon) {
    xs.push_back(pt[0]);
    ys.push_back(pt[1]);
  }
  PlotPolygon(xs, ys, width, color, id, ns);
}

void PlotConvexPolygons(
    const std::vector<std::vector<Eigen::Vector2d>>& polygons, 
    double width,
    Color color, 
    int id, const std::string &ns) {
  int i = 0;
  for (const auto& p : polygons) {
    ++i;
    std::string name_space = ns + std::to_string(i);
    PlotConvexPolygon(p, width, color, id, name_space);
  }
}

void PlotPoint(
    const math::Vec2d& pt, 
    double width,
    Color color, 
    int id, const std::string &ns) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::SPHERE;
  msg.pose.position.x = pt.x();
  msg.pose.position.y = pt.y();
  msg.pose.position.z = 0;
  msg.pose.position.y = pt.y();
  msg.pose.position.z = 0;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = width * 2.0;
  msg.scale.y = width * 2.0;
  msg.scale.z = 0.01;
  msg.color = color.toColorRGBA();
  
  mutex_.lock();
  arr_.markers.push_back(msg);
  mutex_.unlock();
}

void PlotPoints(
    const std::vector<math::Vec2d>& pts, 
    double width,
    Color color, 
    int id, const std::string &ns) {
  int i = 0;
  for (const auto& pt : pts) {
    ++i;
    std::string name_space = ns + std::to_string(i);
    PlotPoint(pt, width, color, id, name_space);
  }
}

void PlotLineSegment(
    const math::LineSegment2d& line, 
    double width,
    Color color, 
    int id, const std::string &ns) {
  std::vector<double> xs, ys;
  xs.push_back(line.start().x());
  xs.push_back(line.end().x());
  ys.push_back(line.start().y());
  ys.push_back(line.end().y());
  Plot(xs, ys, width, color, id, ns);
  // PlotPoint(line.start(), width, color, id, ns + "1");
  // PlotPoint(line.end(), width, Color::Red, id, ns + "2");
}

void PlotLineSegments(
    const std::vector<math::LineSegment2d>& lines, 
    double width,
    Color color, 
    int id, const std::string &ns) {
  int i = 0;
  for (const auto& line : lines) {
    ++i;
    std::string name_space = ns + std::to_string(i);
    PlotLineSegment(line, width, color, id, name_space);
  }
}

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, double max_velocity, double width,
                    const Color &color, int id, const std::string &ns) {
  std::vector<Color> colors(xs.size());
  float h, tmp;
  color.toHSV(h, tmp, tmp);

  for (size_t i = 0; i < xs.size(); i++) {
    double percent = (vs[i] / max_velocity);
    colors[i] = Color::fromHSV(h, percent, 1.0);
  }

  Plot(xs, ys, width, colors, id, ns);
}

void PlotPoints(const Vector &xs, const Vector &ys, double width, const Color &color, int id,
                const std::string &ns) {
  assert(xs.size() == ys.size());

  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.header.stamp = ros::Time();
  msg.ns = ns.empty() ? "Points" : ns;
  msg.id = id >= 0 ? id : arr_.markers.size();

  msg.action = visualization_msgs::Marker::ADD;
  msg.type = visualization_msgs::Marker::POINTS;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = msg.scale.y = width;
  msg.color = color.toColorRGBA();

  for (size_t i = 0; i < xs.size(); i++) {
    geometry_msgs::Point pt;
    pt.x = xs[i];
    pt.y = ys[i];
    msg.points.push_back(pt);
  }

  mutex_.lock();
  arr_.markers.push_back(msg);
  mutex_.unlock();
}

void Trigger() {
  mutex_.lock();
  publisher_.publish(arr_);
  arr_.markers.clear();
  mutex_.unlock();
}

void Clear() {
  mutex_.lock();
  arr_.markers.clear();

  visualization_msgs::MarkerArray arr;
  visualization_msgs::Marker msg;
  msg.header.frame_id = frame_;
  msg.ns = "Markers";

  msg.action = visualization_msgs::Marker::DELETEALL;
  arr.markers.push_back(msg);
  publisher_.publish(arr);
  mutex_.unlock();
}
}
}