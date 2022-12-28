/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

#pragma once

#include <math.h>

#include <cfloat>
#include <memory>
#include <vector>

#include <carla/client/Client.h>
#include <glog/logging.h>

#include "cost_functions.h"
#include "cubic_spiral.h"
#include "planning_params.h"
#include "structs.h"
#include "utils.h"
#include "velocity_profile_generator.h"

namespace cc = carla::client;
namespace cost = cost_functions;
namespace cg = carla::geom;
namespace cf = cost_functions;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

using Waypoint = cc::Waypoint;

class MotionPlanner {
 private:
  unsigned short _num_paths;  // number of lateral offset paths to generate.
  float _goal_offset;         // lateral distance between goals.
  float _error_tolerance;
  CubicSpiral _cubic_spiral;

 public:
  MotionPlanner(unsigned short num_paths, float goal_offset,
                float error_tolerance)
      : _num_paths(num_paths),
        _goal_offset(goal_offset),
        _error_tolerance(error_tolerance) {
    _velocity_profile_generator.setup(P_TIME_GAP, P_MAX_ACCEL, P_SLOW_SPEED);
  }

  ~MotionPlanner();

  VelocityProfileGenerator _velocity_profile_generator;
  std::vector<PathPoint> _best_spiral;
  size_t _prev_step_count{0};

  State get_goal_state_in_ego_frame(const State& ego_state,
                                    const State& goal_state);

  std::vector<State> generate_offset_goals_ego_frame(const State& ego_state,
                                                     const State& goal_state);
  std::vector<State> generate_offset_goals_global_frame(
      const State& goal_state);
  std::vector<State> generate_offset_goals(const State& goal_state);

  std::vector<std::vector<PathPoint>> generate_spirals(
      const State& ego_state, const std::vector<State>& goals);

  std::vector<std::vector<PathPoint>> transform_spirals_to_global_frame(
      const std::vector<std::vector<PathPoint>>& spirals,
      const State& ego_state);

  std::vector<int> get_best_spiral_idx(const std::vector<std::vector<PathPoint>>& spirals,
                          const std::vector<State>& obstacles,
                          const State& goal_state);

  bool valid_goal(const State& main_goal, const State& offset_goal);
  bool valid_spiral(const std::vector<PathPoint>& spiral,
                    const State& offset_goal);
  float calculate_cost(const std::vector<PathPoint>& spiral,
                       const std::vector<State>& obstacles,
                       const State& goal);
};