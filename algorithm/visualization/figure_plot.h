#pragma once

#include <string>
#include <map>
#include <cmath>
#include <vector>
#include <iostream>

#include "algorithm/visualization/matplotlibcpp.h"

#include "algorithm/utils/discretized_trajectory.h"
#include "algorithm/params/vehicle_param.h"

namespace plt = matplotlibcpp;

namespace planning {
namespace visualization {

class FigurePlot {
 public:
  FigurePlot() = default;

  void Plot(const DiscretizedTrajectory& d_traj,
            const bool sub_plot = true) {
    if (d_traj.trajectory().empty()) {
      return;
    }
    if (sub_plot) { plt::figure(1); }
    
    std::vector<double> t, s, x, y, theta, v, a, delta, j, delta_rate, kappa;
    std::vector<double> v_max, a_min, a_max, delta_min, delta_max, j_min, j_max, delta_r_min, delta_r_max;

    for (const auto& pt : d_traj.trajectory()) {
      t.push_back(pt.time);
      s.push_back(pt.s);
      x.push_back(pt.x);
      y.push_back(pt.y);
      theta.push_back(pt.theta);
      v.push_back(pt.velocity);
      a.push_back(pt.a); 
      delta.push_back(pt.delta);
      j.push_back(pt.jerk);
      delta_rate.push_back(pt.delta_rate);
      kappa.push_back(pt.kappa);

      v_max.push_back(vehicle_params_.max_velocity);
      a_min.push_back(vehicle_params_.min_acceleration);
      a_max.push_back(vehicle_params_.max_acceleration);
      delta_min.push_back(vehicle_params_.delta_min);
      delta_max.push_back(vehicle_params_.delta_max);
      j_min.push_back(vehicle_params_.jerk_min);
      j_max.push_back(vehicle_params_.jerk_max);
      delta_r_min.push_back(vehicle_params_.delta_rate_min);
      delta_r_max.push_back(vehicle_params_.delta_rate_max);

    }

    if (sub_plot) { 
      plt::subplot(7,1,1);
    } else {
      plt::figure(2);
    }
    plt::named_plot("x-y", x, y, "k-");
    plt::xlabel("x(m)"); plt::ylabel("y(m)");
    plt::legend();

    if (sub_plot) { 
      plt::subplot(7,1,2);
    } else {
      plt::figure(3);
    }
    plt::named_plot("theta", t, theta, "k-");
    plt::xlabel("t(s)"); plt::ylabel("theta(rad)");
    plt::legend();

    if (sub_plot) { 
      plt::subplot(7,1,3);
    } else {
      plt::figure(4);
    }
    plt::named_plot("v", t, v, "k-");
    plt::named_plot("v_max", t, v_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("v(m/s)");
    plt::legend();

    if (sub_plot) { 
      plt::subplot(7,1,4);
    } else {
      plt::figure(5);
    }
    plt::named_plot("a", t, a, "k-");
    plt::named_plot("a_min", t, a_min, "r-");
    plt::named_plot("a_max", t, a_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("a(m/s2)");
    plt::legend();

    if (sub_plot) { 
      plt::subplot(7,1,5);
    } else {
      plt::figure(6);
    }
    plt::named_plot("delta", t, delta, "k-");
    plt::named_plot("delta_min", t, delta_min, "r-");
    plt::named_plot("delta_max", t, delta_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("delta(rad)");
    plt::legend();

    if (sub_plot) { 
      plt::subplot(7,1,6);
    } else {
      plt::figure(7);
    }
    plt::named_plot("jerk", t, j, "k-");
    plt::named_plot("jerk_min", t, j_min, "r-");
    plt::named_plot("jerk_max", t, j_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("jerk(m/s3)");
    plt::legend();

    if (sub_plot) { 
      plt::subplot(7,1,7);
    } else {
      plt::figure(8);
    }
    plt::named_plot("delta-rate", t, delta_rate, "k-");
    plt::named_plot("delta-rate_min", t, delta_r_min, "r-");
    plt::named_plot("delta-rate_max", t, delta_r_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("delta_rate(rad/s)");
    plt::legend();

    plt::show();
  }

  void Plot(const DiscretizedTrajectory& d_traj_1,
            const DiscretizedTrajectory& d_traj_2,
            const bool sub_plot = true) {
    if (d_traj_1.trajectory().empty() || d_traj_2.trajectory().empty()) {
      return;
    }

    if (sub_plot) { plt::figure(1); }

    std::vector<double> t, s, x, y, theta, v, a, delta, j, delta_rate, kappa;
    std::vector<double> v_max, a_min, a_max, delta_min, delta_max, j_min, j_max, delta_r_min, delta_r_max;

    for (const auto& pt : d_traj_1.trajectory()) {
      t.push_back(pt.time);
      s.push_back(pt.s);
      x.push_back(pt.x);
      y.push_back(pt.y);
      theta.push_back(pt.theta);
      v.push_back(pt.velocity);
      a.push_back(pt.a); 
      delta.push_back(pt.delta);
      j.push_back(pt.jerk);
      delta_rate.push_back(pt.delta_rate);
      kappa.push_back(pt.kappa);

      v_max.push_back(vehicle_params_.max_velocity);
      a_min.push_back(vehicle_params_.min_acceleration);
      a_max.push_back(vehicle_params_.max_acceleration);
      delta_min.push_back(vehicle_params_.delta_min);
      delta_max.push_back(vehicle_params_.delta_max);
      j_min.push_back(vehicle_params_.jerk_min);
      j_max.push_back(vehicle_params_.jerk_max);
      delta_r_min.push_back(vehicle_params_.delta_rate_min);
      delta_r_max.push_back(vehicle_params_.delta_rate_max);
    }

    std::vector<double> t_2, s_2, x_2, y_2, theta_2, v_2, a_2, delta_2, j_2, delta_rate_2, kappa_2;
    for (const auto& pt : d_traj_2.trajectory()) {
      t_2.push_back(pt.time);
      s_2.push_back(pt.s);
      x_2.push_back(pt.x);
      y_2.push_back(pt.y);
      theta_2.push_back(pt.theta);
      v_2.push_back(pt.velocity);
      a_2.push_back(pt.a); 
      delta_2.push_back(pt.delta);
      j_2.push_back(pt.jerk);
      delta_rate_2.push_back(pt.delta_rate);
      kappa_2.push_back(pt.kappa);
    }

    if (sub_plot) {
      plt::subplot(7,1,1);
    } else {
      plt::figure(2);
    }
    plt::named_plot("x_y_1", x, y, "k-");
    plt::named_plot("x_y_2", x_2, y_2, "k--");
    plt::xlabel("x(m)"); plt::ylabel("y(m)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,2);
    } else {
      plt::figure(3);
    }
    plt::named_plot("theta_1", t, theta, "k-");
    plt::named_plot("theta_2", t_2, theta_2, "k--");
    plt::xlabel("t(s)"); plt::ylabel("theta(rad)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,3);
    } else {
      plt::figure(4);
    }
    plt::named_plot("v_1", t, v, "k-");
    plt::named_plot("v_2", t_2, v_2, "k--");
    plt::named_plot("v_max", t, v_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("v(m/s)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,4);
    } else {
      plt::figure(5);
    }
    plt::named_plot("a_1", t, a, "k-");
    plt::named_plot("a_2", t_2, a_2, "k--");
    plt::named_plot("a_min", t, a_min, "r-");
    plt::named_plot("a_max", t, a_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("a(m/s2)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,5);
    } else {
      plt::figure(6);
    }
    plt::named_plot("delta_1", t, delta, "k-");
    plt::named_plot("delta_2", t_2, delta_2, "k--");
    plt::named_plot("delta_min", t, delta_min, "r-");
    plt::named_plot("delta_max", t, delta_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("delta(rad)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,6);
    } else {
      plt::figure(7);
    }
    plt::named_plot("jerk_1", t, j, "k-");
    plt::named_plot("jerk_2", t_2, j_2, "k--");
    plt::named_plot("jerk_min", t, j_min, "r-");
    plt::named_plot("jerk_max", t, j_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("jerk(m/s3)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,7);
    } else {
      plt::figure(8);
    }
    plt::named_plot("delta-rate_1", t, delta_rate, "k-");
    plt::named_plot("delta-rate_2", t_2, delta_rate_2, "k--");
    plt::named_plot("delta-rate_min", t, delta_r_min, "r-");
    plt::named_plot("delta-rate_max", t, delta_r_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("delta_rate(rad/s)");
    plt::legend();

    plt::show();
  }

  void Plot(const DiscretizedTrajectory& d_traj,
            const std::vector<DiscretizedTrajectory>& d_trajs,
            const bool sub_plot = true) {
    if (d_traj.trajectory().empty() || 
        d_trajs.empty() ||
        d_trajs.front().empty()) {
      return;
    }

    if (sub_plot) { plt::figure(1); }

    std::vector<double> t, s, x, y, theta, v, a, delta, j, delta_rate, kappa;
    std::vector<double> v_max, a_min, a_max, delta_min, delta_max, j_min, j_max, delta_r_min, delta_r_max;

    for (const auto& pt : d_traj.trajectory()) {
      t.push_back(pt.time);
      s.push_back(pt.s);
      x.push_back(pt.x);
      y.push_back(pt.y);
      theta.push_back(pt.theta);
      v.push_back(pt.velocity);
      a.push_back(pt.a); 
      delta.push_back(pt.delta);
      j.push_back(pt.jerk);
      delta_rate.push_back(pt.delta_rate);
      kappa.push_back(pt.kappa);

      v_max.push_back(vehicle_params_.max_velocity);
      a_min.push_back(vehicle_params_.min_acceleration);
      a_max.push_back(vehicle_params_.max_acceleration);
      delta_min.push_back(vehicle_params_.delta_min);
      delta_max.push_back(vehicle_params_.delta_max);
      j_min.push_back(vehicle_params_.jerk_min);
      j_max.push_back(vehicle_params_.jerk_max);
      delta_r_min.push_back(vehicle_params_.delta_rate_min);
      delta_r_max.push_back(vehicle_params_.delta_rate_max);
    }

    if (sub_plot) {
      plt::subplot(7,1,1);
    } else {
      plt::figure(2);
    }
    plt::named_plot("x-y", x, y, "k-");
    int i = 0;
    for (const auto& d_traj : d_trajs) {
      x.clear(); y.clear();
      for (const auto& pt : d_traj.trajectory()) {
        x.push_back(pt.x);
        y.push_back(pt.y);
      }
      std::string label_name = "x-y_" + std::to_string(i);
      plt::named_plot(label_name, x, y, Markers(i));
      ++i;
    }
    plt::xlabel("x(m)"); plt::ylabel("y(m)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,2);
    } else {
      plt::figure(3);
    }
    plt::named_plot("theta", t, theta, "k-");
    i = 0;
    for (const auto& d_traj : d_trajs) {
      t.clear(); theta.clear();
      for (const auto& pt : d_traj.trajectory()) {
        t.push_back(pt.time);
        theta.push_back(pt.theta);
      }
      std::string label_name = "theta_" + std::to_string(i);
      plt::named_plot(label_name, t, theta, Markers(i));
      ++i;
    }
    plt::xlabel("t(s)"); plt::ylabel("theta(rad)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,3);
    } else {
      plt::figure(4);
    }
    plt::named_plot("v", t, v, "k-");
    i = 0;
    for (const auto& d_traj : d_trajs) {
      t.clear(); v.clear();
      for (const auto& pt : d_traj.trajectory()) {
        t.push_back(pt.time);
        v.push_back(pt.velocity);
      }
      std::string label_name = "v_" + std::to_string(i);
      plt::named_plot(label_name, t, v, Markers(i));
      ++i;
    }
    plt::named_plot("v_max", t, v_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("v(m/s)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,4);
    } else {
      plt::figure(5);
    }
    plt::named_plot("a", t, a, "k-");
    i = 0;
    for (const auto& d_traj : d_trajs) {
      t.clear(); a.clear();
      for (const auto& pt : d_traj.trajectory()) {
        t.push_back(pt.time);
        a.push_back(pt.a);
      }
      std::string label_name = "a_" + std::to_string(i);
      plt::named_plot(label_name, t, theta, Markers(i));
      ++i;
    }
    plt::named_plot("a_min", t, a_min, "r-");
    plt::named_plot("a_max", t, a_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("a(m/s2)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,5);
    } else {
      plt::figure(6);
    }
    plt::named_plot("delta", t, delta, "k-");
    i = 0;
    for (const auto& d_traj : d_trajs) {
      t.clear(); delta.clear();
      for (const auto& pt : d_traj.trajectory()) {
        t.push_back(pt.time);
        delta.push_back(pt.delta);
      }
      std::string label_name = "delta_" + std::to_string(i);
      plt::named_plot(label_name, t, delta, Markers(i));
      ++i;
    }
    plt::named_plot("delta_min", t, delta_min, "r-");
    plt::named_plot("delta_max", t, delta_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("delta(rad)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,6);
    } else {
      plt::figure(7);
    }
    plt::named_plot("jerk", t, j, "k-");
    i = 0;
    for (const auto& d_traj : d_trajs) {
      t.clear(); j.clear();
      for (const auto& pt : d_traj.trajectory()) {
        t.push_back(pt.time);
        j.push_back(pt.jerk);
      }
      std::string label_name = "jerk_" + std::to_string(i);
      plt::named_plot(label_name, t, j, Markers(i));
      ++i;
    }
    plt::named_plot("jerk_min", t, j_min, "r-");
    plt::named_plot("jerk_max", t, j_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("jerk(m/s3)");
    plt::legend();

    if (sub_plot) {
      plt::subplot(7,1,7);
    } else {
      plt::figure(8);
    }
    plt::named_plot("delta-rate", t, delta_rate, "k-");
    i = 0;
    for (const auto& d_traj : d_trajs) {
      t.clear(); delta_rate.clear();
      for (const auto& pt : d_traj.trajectory()) {
        t.push_back(pt.time);
        delta_rate.push_back(pt.delta_rate);
      }
      std::string label_name = "delta_rate_" + std::to_string(i);
      plt::named_plot(label_name, t, delta_rate, Markers(i));
      ++i;
    }
    plt::named_plot("delta-rate_min", t, delta_r_min, "r-");
    plt::named_plot("delta-rate_max", t, delta_r_max, "r-");
    plt::xlabel("t(s)"); plt::ylabel("delta_rate(rad/s)");
    plt::legend();

    plt::show();
  }

 private:
  std::string Markers(const int num) {
    int index = std::max(0, std::min(num, static_cast<int>(markers_.size() - 1)));
    std::cout << "num: " << num <<  ", index: " << index << std::endl;
    return markers_[index];
  }

 private:
  VehicleParam vehicle_params_;
  std::vector<std::string> markers_{
      "b-", "c-", "g-", "m-", "y-",
      "b--", "c--", "g--", "m--", "y--",
      "b-.", "c-.", "g-.", "m-.", "y-.",
      "b:", "c:", "g:", "m:", "y:",
      "b-+", "c-+", "g-+", "m-+", "y-+",
      "b-*", "c-*", "g-*", "m-*", "y-*"
  };

};

} // namespace visualization
} // namespace planning