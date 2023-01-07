/* ------------------------------------------------------------------------------
 * Project "5.1: Control and Trajectory Tracking for Autonomous Vehicles"
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
 * Implements the vehicle velocity trajectory profile generator.
 *
 * A vehicle velocity trajectory is computed from a given starting speed to a
 * desired final speed. For all trajectories, a maximum permitted acceleration
 * is given by `_a_max` (i.e., a comfortable acceleration / deceleration rate).
 *
 * The velocity trajectory generator works in unison with the behaviour
 * plannner and is defined in "motion_planner.h" and "behavior_planner_FSM.h".
 * A velocity profile is created for each of the possible vehicle states.
 *
 * In the `FOLLOW_LANE` state, the ego-vehicle should either speed up or slow
 * down in order to maintain a target speed.
 *
 * In the `DECEL_TO_STOP` state, the ego-vehicle should decelerate smoothly to
 * a complete stop at a desired stopping point.
 *
 * The order of precendence for handling vehicle state requests is given by:
 *    DECEL_TO_STOP > FOLLOW_VEHICLE > FOLLOW_LANE.
 * Note that in a true velocity planner the coupling of these states would need
 * to be explicitly handled, but in this project we limit the scope such that
 * each is implemented independently from one another.
 *
 * Look at the "structs.h" file for details on the types of manoeuvres / states
 * that the behaviour planner can be in.
 *
 * @class   "velocity_profile_generator.h"
 * @brief   Implements the vehicle velocity trajectory profile generator.
 * @var     _time_gap    Time-gap (s) to use (not implemented).
 * @var     _a_max       Maximum permitted acceleration (m/s^2).
 * @var     _slow_speed  Velocity (m/s) for slow manoeuvre (not implemented).
*/
class VelocityProfileGenerator {
 private:
  // Time-gap (s) to use with velocity profile (not yet implemented)
  double _time_gap;
  // Maximum permitted acceleration (m/s^2)
  double _a_max;
  // Velocity (m/s) to set for the slow manoeuvre (not yet implemented)
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
  // Computes the travel distance needed for given acceleration / deceleration
  double calc_distance(
      const double& v_i,
      const double& v_f,
      const double& a
  ) const;
  // Computes the final velocity of the trajectory given by the state variables
  double calc_final_speed(
      const double& v_i,
      const double& a,
      const double& d
  ) const;
  // Returns the velocity trajectory for deceleration to a full stop
  std::vector<TrajectoryPoint> decelerate_trajectory(
      const std::vector<PathPoint>& spiral,
      const double& start_speed
  ) const;
  // Returns the velocity trajectory for following a lead vehicle
  std::vector<TrajectoryPoint> follow_trajectory(
      const std::vector<PathPoint>& spiral,
      const double& start_speed,
      const double& desired_speed,
      const State& lead_car_state
  ) const;
  // Returns the velocity trajectory for nominal speed tracking
  std::vector<TrajectoryPoint> nominal_trajectory(
      const std::vector<PathPoint>& spiral,
      const double& start_speed,
      double const& desired_speed
  ) const;
  // Returns the trajectory generated for the given state variables
  std::vector<TrajectoryPoint> generate_trajectory(
      const std::vector<PathPoint>& spiral,
      const double& desired_speed,
      const State& ego_state,
      const State& lead_car_state,
      const Maneuver& maneuver
  ) const;
};
