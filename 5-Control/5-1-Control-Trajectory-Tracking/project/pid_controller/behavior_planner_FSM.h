/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file behavior_planner_FSM.h
 **/

#pragma once

#include <chrono>
#include <cmath>

#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/geom/Transform.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include "planning_params.h"
#include "structs.h"
#include "utils.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace cr = carla::road;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace utils;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

using Waypoint = cc::Waypoint;

class BehaviorPlannerFSM {
 private:
  double _lookahead_time;
  double _lookahead_distance_min;
  double _lookahead_distance_max;
  double _speed_limit;
  double _stop_threshold_speed;
  double _req_stop_time;
  double _reaction_time;
  double _max_accel;
  double _stop_line_buffer{-1.0};
  cr::JuncId _prev_junction_id{-1};
  // double _follow_lead_vehicle_lookahead{-1.0};

  std::chrono::time_point<std::chrono::high_resolution_clock> _start_stop_time;

  Maneuver _active_maneuver{FOLLOW_LANE};
  State _goal;

 public:
  BehaviorPlannerFSM(double lookahead_time, double lookahead_distance_min,
                     double lookahead_distance_max, double speed_limit,
                     double stop_threshold_speed, double req_stop_time,
                     double reaction_time, double max_accel,
                     double stop_line_buffer)
      : _lookahead_time(lookahead_time),
        _lookahead_distance_min(lookahead_distance_min),
        _lookahead_distance_max(lookahead_distance_max),
        _speed_limit(speed_limit),
        _stop_threshold_speed(stop_threshold_speed),
        _req_stop_time(req_stop_time),
        _reaction_time(reaction_time),
        _max_accel(max_accel),
        _stop_line_buffer(stop_line_buffer){};

  ~BehaviorPlannerFSM(){};

  double get_look_ahead_distance(const State& ego_state);

  State get_closest_waypoint_goal(const State& ego_state,
                                                const SharedPtr<cc::Map>& map,
                                                const float& lookahead_distance,
                                                bool& is_goal_junction);

  State get_goal(const State& ego_state, SharedPtr<cc::Map> map);

  State state_transition(const State& ego_state,
                         State goal,
                         bool& is_goal_in_junction,
                         string tl_state);

  Maneuver get_active_maneuver() { return _active_maneuver; };
};
