/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file velocity_trajectory_generator.h
 **/

#pragma once

#include <vector>

#include <carla/client/Client.h>
#include <glog/logging.h>

#include "structs.h"
#include "utils.h"

namespace cc = carla::client;
namespace cg = carla::geom;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

using Waypoint = cc::Waypoint;

class VelocityProfileGenerator {
 private:
  double _time_gap;
  double _a_max;
  double _slow_speed;

  // std::vector<TrajectoryPoint> _prev_trajectory;

 public:
  VelocityProfileGenerator();

  ~VelocityProfileGenerator();

  void setup(const double& time_gap, const double& a_max,
             const double& slow_speed);

  std::vector<TrajectoryPoint> generate_trajectory(
      const std::vector<PathPoint>& spiral, const double& desired_speed,
      const State& ego_state, const State& lead_car_state,
      const Maneuver& maneuver) const;

  std::vector<TrajectoryPoint> decelerate_trajectory(
      const std::vector<PathPoint>& spiral, const double& start_speed) const;

  std::vector<TrajectoryPoint> follow_trajectory(
      const std::vector<PathPoint>& spiral, const double& start_speed,
      const double& desired_speed, const State& lead_car_state) const;

  std::vector<TrajectoryPoint> nominal_trajectory(
      const std::vector<PathPoint>& spiral, const double& start_speed,
      double const& desired_speed) const;

  double calc_distance(const double& v_i, const double& v_f,
                       const double& a) const;

  double calc_final_speed(const double& v_i, const double& a,
                          const double& d) const;
};
