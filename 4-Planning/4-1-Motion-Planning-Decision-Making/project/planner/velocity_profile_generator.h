/* ------------------------------------------------------------------------------
 * Project "4.1: Motion Planning and Decision Making for Autonomous Vehicles"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the velocity profile generator.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "structs.h"
#include "utils.h"
#include <carla/client/Client.h>
#include <glog/logging.h>
#include <vector>
#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <thread>

namespace cc = carla::client;
namespace cg = carla::geom;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

using Waypoint = cc::Waypoint;

/*
This class computes a velocity trajectory from a starting speed to a desired
speed. It works in unison with the Behavioral plannner  as it needs to build a
velocity profile for each of the states that the vehicle can be in.
In the "Follow_lane" state we need to either speed up or speed down to maintain
a speed target. In the "decel_to_stop" state we need to create a profile that
allows us to decelerate smoothly to a stop line.

The order of precedence for handling these cases is stop sign handling and then
nominal lane maintenance. In a real velocity planner you would need to handle
the coupling between these states, but for simplicity this project can be
implemented by isolating each case.

For all trajectories, the required acceleration is given by _a_max (confortable
accel).
Look at the structs.h for details on the types of manuevers/states that the
behavior planner can be in.
*/
class VelocityProfileGenerator {
 private:
  double _time_gap;
  double _a_max;
  double _slow_speed;
  // std::vector<TrajectoryPoint> _prev_trajectory;

 public:
  VelocityProfileGenerator();
  ~VelocityProfileGenerator();

  void setup(
      const double& time_gap, 
      const double& a_max,
      const double& slow_speed
  );
  std::vector<TrajectoryPoint> generate_trajectory(
      const std::vector<PathPoint>& spiral, 
      const double& desired_speed,
      const State& ego_state, 
      const State& lead_car_state,
      const Maneuver& maneuver
  ) const;
  std::vector<TrajectoryPoint> decelerate_trajectory(
      const std::vector<PathPoint>& spiral, 
      const double& start_speed
  ) const;
  std::vector<TrajectoryPoint> follow_trajectory(
      const std::vector<PathPoint>& spiral, 
      const double& start_speed,
      const double& desired_speed, 
      const State& lead_car_state
  ) const;
  std::vector<TrajectoryPoint> nominal_trajectory(
      const std::vector<PathPoint>& spiral, 
      const double& start_speed,
      double const& desired_speed
  ) const;
  double calc_distance(
      const double& v_i, 
      const double& v_f,
      const double& a
  ) const;
  double calc_final_speed(
      const double& v_i, 
      const double& a,
      const double& d
  ) const;
};
