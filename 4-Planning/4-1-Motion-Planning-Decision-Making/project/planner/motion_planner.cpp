/* ------------------------------------------------------------------------------
 * Project "4.1: Motion Planning and Decision Making for Autonomous Vehicles"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the motion planner.
 * ----------------------------------------------------------------------------
 */

#include "motion_planner.h"


MotionPlanner::~MotionPlanner() {}


/* Computes the goal-state coordinate transformations in the ego-vehicle frame. 
 *
 * @param    ego_state   Ego-vehicle state (pose) to transform offsets w.r.t.
 * @param    goal_state  Goal-state pose to transform into ego-vehicle frame.
 * @returns  Goal-state pose defined in the ego-vehicle reference frame.
 *
 */
State MotionPlanner::get_goal_state_in_ego_frame(
    const State& ego_state,
    const State& goal_state
) {
  // Let's start by making a copy of the goal state (global reference frame)
  auto goal_state_ego_frame = goal_state;
  // Translate so the ego state is at the origin in the new frame.
  // This is done by subtracting the ego_state from the goal_ego_.
  goal_state_ego_frame.location.x -= ego_state.location.x;
  goal_state_ego_frame.location.y -= ego_state.location.y;
  goal_state_ego_frame.location.z -= ego_state.location.z;
  /* Rotate such that the ego state has zero heading/yaw in the new frame.
     We are rotating by -ego_state "yaw" to ensure the ego vehicle's
     current yaw corresponds to theta = 0 in the new local frame.
     Recall that the general rotation matrix around the Z axix is:
     [cos(theta) -sin(theta)
     sin(theta)  cos(theta)]
  */
  auto theta_rad = -ego_state.rotation.yaw;
  auto cos_theta = std::cos(theta_rad);
  auto sin_theta = std::sin(theta_rad);
  goal_state_ego_frame.location.x = (
      cos_theta * goal_state_ego_frame.location.x -
      sin_theta * goal_state_ego_frame.location.y
  );
  goal_state_ego_frame.location.y = (
      sin_theta * goal_state_ego_frame.location.x +
      cos_theta * goal_state_ego_frame.location.y
  );
  // Compute the goal yaw in the local frame by subtracting off the
  // current ego yaw from the goal waypoint heading/yaw.
  goal_state_ego_frame.rotation.yaw += theta_rad;
  // Ego speed is the same in both coordenates
  // the Z coordinate does not get affected by the rotation.
  // Let's make sure the yaw is within [-180, 180] or [-pi, pi] so the optimizer
  // works.
  goal_state_ego_frame.rotation.yaw = utils::keep_angle_range_rad(
      goal_state_ego_frame.rotation.yaw, -M_PI, M_PI
  );
  // if (goal_state_ego_frame.rotation.yaw < -M_PI) {
  //   goal_state_ego_frame.rotation.yaw += (2 * M_PI);
  // } else if (goal_state_ego_frame.rotation.yaw > M_PI) {
  //   goal_state_ego_frame.rotation.yaw -= (2 * M_PI);
  // }
  return goal_state_ego_frame;
}


/* Helper function to return goal-offset states (poses) in ego-vehicle frame.
 *
 * Here the goal-offsets are computed w.r.t. the given `goal_state`. 
 * Then, the goal-offset coordinates are transformed into the ego-vehicle
 * reference frame which is defined w.r.t. the `ego_state`.
 * 
 * @param    ego_state    Ego-vehicle state (pose) to transform offsets w.r.t.
 * @param    goal_state   Goal-state pose to compute the offsets w.r.t.
 * @returns  Offset-goals whose coordinates are defined in ego-vehicle frame.
 */
std::vector<State> MotionPlanner::generate_offset_goals_ego_frame(
    const State& ego_state, 
    const State& goal_state
) {
  // Let's transform the "main" goal (goal state) into ego reference frame
  auto goal_state_ego_frame = get_goal_state_in_ego_frame(
      ego_state, 
      goal_state
  );
  return generate_offset_goals(goal_state_ego_frame);
}

/* Helper function to return goal-offset states (poses) in global frame.
 * 
 * NOTE: As of now, the `generate_offset_goals` computes the coordinates of
 * the offset-goal pose in the global coordinate frame. In other words, no
 * coordinate transformations need to be performed inside this function.
 * 
 * @param    goal_state  Goal-state pose to compute the offsets w.r.t.
 * @returns  Offset-goals whose coordinates are defined in global frame.
 */
std::vector<State> MotionPlanner::generate_offset_goals_global_frame(
    const State& goal_state
) {
  return generate_offset_goals(goal_state);
}


/* Generates pre-defined number of paths which are offset from the goal-state.
 *
 * Each path is offset along the line perpendicular to the heading of the
 * original `goal_state`. Then, the generated goal-offset paths are checked
 * to make sure their final waypoints are within a given distance to the
 * original goal-state location. If the generated goal-offset passes the
 * check, then it is added to the returned `goals_offset` vector.
 *
 * @param    goal_state    Goal-state pose to compute the offset w.r.t.
 * @returns  goals_offset  Set of poses offset from the goal-state.
 */
std::vector<State> MotionPlanner::generate_offset_goals(
    const State& goal_state
) {
  // Now we need to gernerate "_num_paths" goals offset from the center goal at
  // a distance "_goal_offset".
  std::vector<State> goals_offset;
  // Align the new goal-state along the perpendicular line from the heading
  // of the original goal-state (i.e., add $\pi / 2$ to the original yaw angle)
  auto yaw = goal_state.rotation.yaw + M_PI_2;
  // LOG(INFO) << "MAIN GOAL";
  // LOG(INFO) << "x: " << goal_state.location.x << " y: " <<
  // goal_state.location.y
  //          << " z: " << goal_state.location.z
  //          << " yaw (rad): " << goal_state.rotation.yaw;
  // LOG(INFO) << "OFFSET GOALS";
  // LOG(INFO) << "ALL offset yaw (rad): " << yaw;
  for (int i = 0; i < _num_paths; ++i) {
    auto goal_offset = goal_state;
    float offset = (i - (int)(_num_paths / 2)) * _goal_offset;
    // LOG(INFO) << "Goal: " << i + 1;
    // LOG(INFO) << "(int)(_num_paths / 2): " << (int)(_num_paths / 2);
    // LOG(INFO) << "(i - (int)(_num_paths / 2)): " << (i - (int)(_num_paths /
    // 2)); LOG(INFO) << "_goal_offset: " << _goal_offset;
    // LOG(INFO) << "offset: " << offset;
    // Set the coordinates of the goal-offset along the $x$- and $y$-axes
    // perpendicular to the original goal-state heading
    goal_offset.location.x += offset * std::cos(yaw);
    goal_offset.location.y += offset * std::sin(yaw);
    // LOG(INFO) << "x: " << goal_offset.location.x
    //          << " y: " << goal_offset.location.y
    //          << " z: " << goal_offset.location.z
    //          << " yaw (rad): " << goal_offset.rotation.yaw;
    if (valid_goal(goal_state, goal_offset)) {
      goals_offset.push_back(goal_offset);
    }
  }
  return goals_offset;
}


/* Returns boolean displacement cost from offset- to goal-state.  
 *
 * @param    main_goal    Goal-state location to evaluate.
 * @param    offset_goal  Goal-offset location to compute displacement w.r.t. 
 * @returns  Boolean cost whether the offset distance exceeds max threshold. 
 */
bool MotionPlanner::valid_goal(
    const State& main_goal,
    const State& offset_goal
) {
  auto max_offset = ((int)(_num_paths / 2) + 1) * _goal_offset;
  // LOG(INFO) << "max offset: " << max_offset;
  auto dist = utils::magnitude(main_goal.location - offset_goal.location);
  // LOG(INFO) << "distance from main goal to offset goal: " << dist;
  return dist < max_offset;
}


/* Returns the indices of the paths which satisfies the possible cases.
 *
 * The index of a path in `spirals` in which a collision occurs will be
 * added to the returned `collisions` vector at the position in the vector
 * where it was examined in the loop (i.e., its position in `spirals`).
 * 
 * Assuming there exists an optimal path (no collisions, lowest cost score),
 * the corresponding index of this path in `spirals` will be added to the
 * last element of the returned `collisions` vector.
 * 
 * If no collisions occur, and no optimal path exists, the returned vector
 * will be empty. If all paths are not optimal, this could be because the
 * final waypoint is not near the goal-state, or the trajectory differs
 * significantly from the intended trajectory. 
 *
 * @param    spirals     Paths to examine for collisions / lowest score.
 * @param    obstacles   Set of objects to evaluate.
 * @param    goal_state  Goal-state / final waypoint positioned in lane centre. 
 * @returns  Set of indices s.t. last element is the index of path with lowest
 *           cost, or empty vector if no collisions or optimal path exists. 
 */
std::vector<int> MotionPlanner::get_best_spiral_idx(
    const std::vector<std::vector<PathPoint>>& spirals,
    const std::vector<State>& obstacles,
    const State& goal_state
) {
  // LOG(INFO) << "Num Spirals: " << spirals.size();
  double best_cost = DBL_MAX;
  std::vector<int> collisions;
  int best_spiral_idx = -1;
  for (size_t i = 0; i < spirals.size(); ++i) {
    double cost = calculate_cost(spirals[i], obstacles, goal_state);
    if (cost < best_cost) {
      // Update the lowest cost value / index found so far
      best_cost = cost;
      best_spiral_idx = i;
    }
    if (cost > DBL_MAX) {
      // Append index of path with collision to position in returned
      // vector corresponding to this path's index in `spirals`
      collisions.push_back(i);
    }
  }
  if (best_spiral_idx != -1) {
    // Append index of best path to back of returned vector
    collisions.push_back(best_spiral_idx);
    return collisions;
  }
  // Return empty vector if `spirals` contains paths which:
  // (1) Do not have any collisions, and
  // (2) Do not have an optimal cost.
  std::vector<int> no_results;
  return no_results;
}


/* Performs a coordinate transformation of each waypoint into the global frame.
 *
 * @param    spiral     Set of paths with the waypoints to transform.
 * @param    ego_state  Reference point of the ego-vehicle to transform w.r.t.
 * @returns  Paths with waypoint coordinates in the global reference frame.
 */
auto MotionPlanner::transform_spirals_to_global_frame(
    const std::vector<std::vector<PathPoint>>& spirals,
    const State& ego_state
) -> std::vector<std::vector<PathPoint>> {
  std::vector<std::vector<PathPoint>> transformed_spirals;
  for (auto spiral : spirals) {
    std::vector<PathPoint> transformed_single_spiral;
    for (auto path_point : spiral) {
      PathPoint new_path_point;
      new_path_point.x = (
          ego_state.location.x 
          + path_point.x * std::cos(ego_state.rotation.yaw) 
          - path_point.y * std::sin(ego_state.rotation.yaw)
      );
      new_path_point.y = (
          ego_state.location.y 
          + path_point.x * std::sin(ego_state.rotation.yaw) 
          + path_point.y * std::cos(ego_state.rotation.yaw)
      );
      new_path_point.theta = (
          path_point.theta 
          + ego_state.rotation.yaw
      );
      transformed_single_spiral.emplace_back(new_path_point);
    }
    transformed_spirals.emplace_back(transformed_single_spiral);
  }
  return transformed_spirals;
}


/* Generates the set of drivable paths to the desired waypoints. 
 *
 * The paths are represented as polynomial spirals, each with a
 * `P_NUM_POINTS_IN_SPIRAL`. The paths generated here follow a set
 * of comfort constraints and are said to be drivable based on their
 * maximum curvature. Each path is defined w.r.t. the starting `ego_state`
 * and one of the final goal-state poses in `goals`.
 * 
 * @param    ego_state  Starting ego-state to generate path from.
 * @param    goals      Set of final ego-state poses to generate path to.
 * @returns  spirals    Set of drivable paths from `ego_state` to `goals`.
 */
auto MotionPlanner::generate_spirals(
    const State& ego_state, 
    const std::vector<State>& goals
) -> std::vector<std::vector<PathPoint>> {
  // Get the starting pose
  // NOTE: ego-vehicle frame has starting point at (0, 0, 0) 
  PathPoint start;
  start.x = ego_state.location.x;
  start.y = ego_state.location.y;
  start.z = ego_state.location.z;
  start.theta = ego_state.rotation.yaw;
  // Define the starting curvature constraints
  start.kappa = 0.0;
  start.s = 0.0;
  start.dkappa = 0.0;
  start.ddkappa = 0.0;
  std::vector<std::vector<PathPoint>> spirals;
  // Create a path to each goal-state in `goals`
  for (auto goal : goals) {
    PathPoint end;
    // Get the final goal-state pose
    end.x = goal.location.x;
    end.y = goal.location.y;
    end.z = goal.location.z;
    end.theta = goal.rotation.yaw;
    // Compute the length of the path
    // Here, we assume zero curvature along the path
    end.kappa = 0.0;
    end.dkappa = 0.0;
    end.ddkappa = 0.0;
    end.s = std::sqrt((end.x * end.x) + (end.y * end.y));
    if (_cubic_spiral.GenerateSpiral(start, end)) {
      // Path was created successfully, save the generated path 
      std::vector<PathPoint>* spiral = new std::vector<PathPoint>;
      auto ok = _cubic_spiral.GetSampledSpiral(
          P_NUM_POINTS_IN_SPIRAL, 
          spiral
      );
      if (ok && valid_spiral(*spiral, goal)) {
        // Generated path is valid and was retrieved successfully
        // Save the path to the `spirals` vector
        // LOG(INFO) << "Spiral Valid ";
        spirals.push_back(*spiral);
      } 
      else {
        // LOG(INFO) << "Spiral Invalid ";
      }
    } 
    else {
      // LOG(INFO) << "Spiral Generation FAILED! ";
    }
  }
  return spirals;
}


/* Checks if the path ends near the desired goal-offset.
 *
 * Returns `true` if distance from last waypoint in `spiral` path
 * is within a fixed distance (here, `0.1` metres) from the desired
 * `offset_goal` position in 2D.
 * 
 * @param    spiral        Path containing the waypoints to evaluate.
 * @param    offset_goal   Desired stopping point for this path.
 * @returns  Boolean value indicating whether the spiral (path) ends near goal.
 */
bool MotionPlanner::valid_spiral(
    const std::vector<PathPoint>& spiral,
    const State& offset_goal
) {
  auto n = spiral.size();
  auto delta_x = (offset_goal.location.x - spiral[n - 1].x);
  auto delta_y = (offset_goal.location.y - spiral[n - 1].y);
  auto dist = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
  // auto dist = utils::magnitude(spiral[spiral.size() - 1].location -
  //                              offset_goal.location);
  // LOG(INFO) << "Distance from Spiral end to offset_goal: " << dist;
  return (dist < 0.1);
}


/* Evaluates the cost functions for the given state variables.
 *
 * @param    spiral     Path containing the waypoints to evaluate.
 * @param    obstacles  Set of object states to evaluate.
 * @param    goal       Goal-state positioned in the lane centre-line.
 * @returns  cost       Summed total cost w.r.t. the given state variables.
 */
float MotionPlanner::calculate_cost(
    const std::vector<PathPoint>& spiral,
    const std::vector<State>& obstacles,
    const State& goal
) {
  // LOG(INFO) << "Starting spiral cost calc";
  // Initialize cost to 0.0
  float cost = 0.0;
  cost += cf::collision_circles_cost_spiral(spiral, obstacles);
  cost += cf::close_to_main_goal_cost_spiral(spiral, goal);
  // LOG(INFO) << "Path Cost: " << cost;
  return cost;
}
