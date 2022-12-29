/* ------------------------------------------------------------------------------
 * Project "4.1: Motion Planning and Decision Making for Autonomous Vehicles"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the motion planner.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "cost_functions.h"
#include "cubic_spiral.h"
#include "planning_params.h"
#include "structs.h"
#include "utils.h"
#include "velocity_profile_generator.h"
#include <carla/client/Client.h>
#include <glog/logging.h>
#include <math.h>
#include <cfloat>
#include <memory>
#include <vector>

template <typename T>
using SharedPtr = boost::shared_ptr<T>;
using Waypoint = carla::client::Waypoint;


class MotionPlanner {
 private:
  // Number of paths with lateral offset to generate 
  unsigned short _num_paths;
  // Lateral distance between goals
  float _goal_offset;
  // Amount of error to tolerate (not yet implemented)
  float _error_tolerance;
  CubicSpiral _cubic_spiral;

 public:
  MotionPlanner(
      // Number of goal-offset paths to generate
      unsigned short num_paths,
      // Relative lateral path offset from the goal-state to tolerate
      float goal_offset,
      // Amount of error to tolerate (not yet implemented)
      float error_tolerance
  )  
      : _num_paths(num_paths),
        _goal_offset(goal_offset),
        _error_tolerance(error_tolerance) {
    _velocity_profile_generator.setup(P_TIME_GAP, P_MAX_ACCEL, P_SLOW_SPEED);
  }
  ~MotionPlanner();
  VelocityProfileGenerator _velocity_profile_generator;
  std::vector<PathPoint> _best_spiral;
  size_t _prev_step_count{0};
  // Computes the goal-state coordinate transformations in the ego-vehicle frame
  State get_goal_state_in_ego_frame(
      const State& ego_state,
      const State& goal_state
  );
  // Helper function to transform goal-offset states in ego-vehicle frame
  std::vector<State> generate_offset_goals_ego_frame(
      const State& ego_state,
      const State& goal_state
  );
  // Helper function to transform goal-offset states in global frame
  std::vector<State> generate_offset_goals_global_frame(
      const State& goal_state
  );
  // Generates pre-defined number of paths which are offset from the goal-state
  std::vector<State> generate_offset_goals(
      const State& goal_state
  );
  // Generates the set of drivable paths to the desired waypoints
  std::vector<std::vector<PathPoint>> generate_spirals(
      const State& ego_state, 
      const std::vector<State>& goals
  );
  // Performs a coordinate transformation of each waypoint into the global frame
  std::vector<std::vector<PathPoint>> transform_spirals_to_global_frame(
      const std::vector<std::vector<PathPoint>>& spirals,
      const State& ego_state
  );
  // Returns the indices of the paths which satisfies the possible cases
  std::vector<int> get_best_spiral_idx(
      const std::vector<std::vector<PathPoint>>& spirals,
      const std::vector<State>& obstacles,
      const State& goal_state
  );
  // Returns the boolean displacement cost from offset- to goal-state
  bool valid_goal(
      const State& main_goal, 
      const State& offset_goal
  );
  // Checks if the path ends near the desired goal-offset
  bool valid_spiral(
      const std::vector<PathPoint>& spiral,
      const State& offset_goal
  );
  // Evaluates the cost functions for the given state variables
  float calculate_cost(
      const std::vector<PathPoint>& spiral,
      const std::vector<State>& obstacles,
      const State& goal
  );
};