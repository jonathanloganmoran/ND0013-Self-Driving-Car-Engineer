/* ------------------------------------------------------------------------------
 * Project "4.1: Motion Planning and Decision Making for Autonomous Vehicles"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for FSM-based behaviour planner.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "velocity_profile_generator.h"
#include "planning_params.h"
#include "structs.h"
#include "utils.h"
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/geom/Transform.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace utils;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

using Waypoint = carla::client::Waypoint;


class BehaviorPlannerFSM {
 private:
  // Time allocated to executing a given manoeuvre
  // e.g., a complete stop
  double _lookahead_time;
  // Minimum distance needed to make a complete stop
  double _lookahead_distance_min;
  // Maximum distance to make a complete stop
  double _lookahead_distance_max;
  // Speed limit to set for the nominal state
  double _speed_limit;
  double _stop_threshold_speed;
  // Time required for the vehicle to wait before proceeding through junction
  double _req_stop_time;
  double _reaction_time;
  double _max_accel;
  // Distance amount to subtract from the goal-state position
  // in order to e.g., prevent collisions or cross stop / intersection lines
  double _stop_line_buffer{-1.0};
  carla::road::JuncId _prev_junction_id{-1};
  // double _follow_lead_vehicle_lookahead{-1.0};
  std::chrono::time_point<std::chrono::high_resolution_clock> _start_stop_time;
  // Ego-vehicle manoeuvre state (evaluated by the FSM)
  Maneuver _active_maneuver{FOLLOW_LANE};
  State _goal;

 public:
  BehaviorPlannerFSM(
      double lookahead_time,
      double lookahead_distance_min,
      double lookahead_distance_max,
      double speed_limit,
      double stop_threshold_speed,
      double req_stop_time,
      double reaction_time,
      double max_accel,
      double stop_line_buffer
  )
      : _lookahead_time(lookahead_time),
        _lookahead_distance_min(lookahead_distance_min),
        _lookahead_distance_max(lookahead_distance_max),
        _speed_limit(speed_limit),
        _stop_threshold_speed(stop_threshold_speed),
        _req_stop_time(req_stop_time),
        _reaction_time(reaction_time),
        _max_accel(max_accel),
        _stop_line_buffer(stop_line_buffer) {};
  ~BehaviorPlannerFSM() {};
  State state_transition(
      const State& ego_state,
      State goal,
      bool& is_goal_in_junction,
      std::string tl_state
  );
  State get_closest_waypoint_goal(
      const State& ego_state,
      const SharedPtr<carla::client::Map>& map,
      const float& lookahead_distance,
      bool& is_goal_junction
  );
  double get_look_ahead_distance(
      const State& ego_state
  );
  State get_goal(
      const State& ego_state,
      SharedPtr<carla::client::Map> map
  );
  Maneuver get_active_maneuver() { return _active_maneuver; };
};
