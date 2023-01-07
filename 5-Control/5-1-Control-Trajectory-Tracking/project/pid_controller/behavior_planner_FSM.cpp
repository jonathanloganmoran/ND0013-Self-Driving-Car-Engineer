/* ------------------------------------------------------------------------------
 * Project "5.1: Control and Trajectory Tracking for Autonomous Vehicles"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the FSM-based behaviour planner.
 * ----------------------------------------------------------------------------
 */

#include "behavior_planner_FSM.h"


/* Updates the ego-vehicle state by evaluating the state transition function.
 *
 * If the desired goal-state is located inside an intersection / junction, then
 * the updated `goal` will be placed at a position behind the junction by the
 * pre-defined `_stop_line_buffer` amount. Otherwise, we assume the vehicle is
 * in a nominal state and can move freely. In this case, goal-state velocity is
 * set to the pre-defined `_speed_limit` w.r.t. the 2D components of the vehicle
 * heading.
 *
 * If `STOPPED` at a controlled intersection (i.e., with traffic light), the
 * ego-vehicle will proceed to a `FOLLOW_LANE` state once the traffic light is
 * not "Red" and a pre-defined amount of `_req_stop_time` has passed.
 *
 * NOTE: We assume that the motion controller is not yet implemented, therefore we
 * use a distance threshold to stop the vehicle in the `DECEL_TO_STOP` state.
 *
 * @param    ego_state            Current ego-vehicle state.
 * @param    goal                 Pose of the goal-state.
 * @param    is_goal_in_junction  Whether the goal-state is in a junction.
 * @param    tl_state             State of the traffic light.
 * @returns  goal                 Goal-state updated w.r.t. the current state.
 */
State BehaviorPlannerFSM::state_transition(
    const State& ego_state,
    State goal,
    bool& is_goal_in_junction,
    std::string tl_state
) {
  // Check with the Behavior Planner to see what we are going to do
  // and where our next goal is
  goal.acceleration.x = 0;
  goal.acceleration.y = 0;
  goal.acceleration.z = 0;
  if (_active_maneuver == FOLLOW_LANE) {
    // LOG(INFO) << "BP- IN FOLLOW_LANE STATE";
    if (is_goal_in_junction) {
      // LOG(INFO) << "BP - goal in junction";
      _active_maneuver = DECEL_TO_STOP;
      // LOG(INFO) << "BP - changing to DECEL_TO_STOP";
      // Let's backup a "buffer" distance behind the "STOP" point
      // LOG(INFO) << "BP- original STOP goal at: " << goal.location.x << ", "
      //          << goal.location.y;
      // Compute the "backed up" location of the goal-state
      // The goal location is placed behind the desired stopping point
      auto ang = goal.rotation.yaw + M_PI;
      goal.location.x += _stop_line_buffer * std::cos(ang);
      goal.location.y += _stop_line_buffer * std::sin(ang);
      // LOG(INFO) << "BP- new STOP goal at: " << goal.location.x << ", "
      //          << goal.location.y;
      // Set the goal-state velocity for the complete stop manoeuvre
      goal.velocity.x = 0.0;
      goal.velocity.y = 0.0;
      goal.velocity.z = 0.0;
    }
    else {
      // Compute the goal-state velocity for the nominal state
      // The velocity components are set w.r.t. the pre-defined speed limit
      goal.velocity.x = _speed_limit * std::cos(goal.rotation.yaw);
      goal.velocity.y = _speed_limit * std::sin(goal.rotation.yaw);
      goal.velocity.z = 0;
    }
  }
  else if (_active_maneuver == DECEL_TO_STOP) {
    // LOG(INFO) << "BP- IN DECEL_TO_STOP STATE";
    // Track the previous goal-state, i.e., set new goal-state to previous
    // in order to keep / maintain the goal at the desired stopping point
    goal = _goal;
    // Since we are not using a motion controller (yet), we need an alternative
    // way to control the vehicle speed in the `DECEL_TO_STOP` state
    // Therefore, we will use distance instead of speed in order to make sure
    // the ego-vehicle comes to a complete stop at the stopping point
    auto distance_to_stop_sign = utils::magnitude(
        goal.location - ego_state.location
    );
    // LOG(INFO) << "Ego distance to stop line: " << distance_to_stop_sign;
    if (distance_to_stop_sign <= P_STOP_THRESHOLD_DISTANCE) {
      // Update the ego-vehicle state to `STOPPED`
      _active_maneuver = STOPPED;
      _start_stop_time = std::chrono::high_resolution_clock::now();
      // LOG(INFO) << "BP - changing to STOPPED";
    }
  }
  else if (_active_maneuver == STOPPED) {
    // LOG(INFO) << "BP- IN STOPPED STATE";
    // Track the previous goal-state, i.e., set new goal-state to previous
    // in order to keep / maintain the `STOPPED` goal-state
    goal = _goal;
    long long stopped_secs = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::high_resolution_clock::now() - _start_stop_time
    ).count();
    // LOG(INFO) << "BP- Stopped for " << stopped_secs << " secs";
    if (stopped_secs >= _req_stop_time && tl_state.compare("Red") != 0) {
      // Since traffic light is no longer "Red" and safety duration has elapsed
      // Move the ego-vehicle to a `FOLLOW_LANE` state
      _active_maneuver = FOLLOW_LANE;
      // LOG(INFO) << "BP - changing to FOLLOW_LANE";
    }
  }
  _goal = goal;
  return goal;
}


/* Returns the closest waypoint to the goal-state.
 *
 * @param    ego_state            Current ego-vehicle state.
 * @param    map                  All waypoints known to the ego-vehicle.
 * @param    lookahead_distance   Distance to travel for complete stop.
 * @param    is_goal_in_junction  Whether the goal-state is in a junction.
 * @returns  waypoint             Closest waypoint to the goal-state.
 */
State BehaviorPlannerFSM::get_closest_waypoint_goal(
    const State& ego_state,
    const SharedPtr<carla::client::Map>& map,
    const float& lookahead_distance,
    bool& is_goal_junction
) {
  // Nearest waypoint situated in the centre of a driving lane
  auto waypoint_0 = map->GetWaypoint(ego_state.location);
  if (_active_maneuver == DECEL_TO_STOP || _active_maneuver == STOPPED) {
    State waypoint;
    auto wp_transform = waypoint_0->GetTransform();
    waypoint.location = wp_transform.location;
    waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
    waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
    waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
    return waypoint;
  }
  // Waypoints at a lookahead distance
  // NOTE: `GetNext(d)` creates a list of waypoints at an approximate distance
  // "d" in the direction of the lane.
  // The list contains one waypoint for each deviation possible.
  // NOTE 2: `GetNextUntilLaneEnd(d)` returns a list of waypoints a distance
  // "d" apart.
  // The list goes from the current waypoint to the end of its lane.
  auto lookahead_waypoints = waypoint_0->GetNext(lookahead_distance);
  auto n_wp = lookahead_waypoints.size();
  if (n_wp == 0) {
    // LOG(INFO) << "Goal wp is a nullptr";
    State waypoint;
    return waypoint;
  }
  // LOG(INFO) << "BP - Num of Lookahead waypoints: " << n_wp;
  waypoint_0 = lookahead_waypoints[lookahead_waypoints.size() - 1];
  is_goal_junction = waypoint_0->IsJunction();
  // LOG(INFO) << "BP - Is Last wp in junction? (0/1): " << is_goal_junction;
  auto cur_junction_id = waypoint_0->GetJunctionId();
  if (is_goal_junction) {
    if (cur_junction_id == _prev_junction_id) {
      // LOG(INFO) << "BP - Last wp is in same junction as ego. Junction ID: "
      //          << _prev_junction_id;
      is_goal_junction = false;
    } else {
      // LOG(INFO) << "BP - Last wp is in different junction than ego. Junction
      // ID: "
      //          << cur_junction_id;
      _prev_junction_id = cur_junction_id;
    }
  }
  State waypoint;
  auto wp_transform = waypoint_0->GetTransform();
  waypoint.location = wp_transform.location;
  waypoint.rotation.yaw = utils::deg2rad(wp_transform.rotation.yaw);
  waypoint.rotation.pitch = utils::deg2rad(wp_transform.rotation.pitch);
  waypoint.rotation.roll = utils::deg2rad(wp_transform.rotation.roll);
  return waypoint;
}


/* Computes the look-ahead distance.
 *
 * The look-ahead distance is the longitudinal distance needed to travel
 * in order for the vehicle to come to a complete stop using a comfortable
 * deceleration w.r.t. a given velocity and look-ahead time.
 *
 * From the 1D rectilinear motion equation for distance, we have:
 *  $ d = 0.5 * a * t^2 + v * t$,
 * where $a$ is the relative acceleration w.r.t. comfort, $v$ is the current
 * ego-vehicle velocity, and $t$ is the pre-defined elapsed time to complete
 * the manoeuvre (i.e., the look-ahead time).
 *
 * @param    ego_state  Current state of ego-vehicle (pose and 1D kinematics).
 * @returns  Distance needed to travel in order to come to a complete stop.
 *
 */
double BehaviorPlannerFSM::get_look_ahead_distance(
    const State& ego_state
) {
  auto velocity_mag = utils::magnitude(ego_state.velocity);
  auto accel_mag = utils::magnitude(ego_state.acceleration);
  // Compute the look-ahead distance using the 1D rectilinear motion equation
  auto look_ahead_distance = (
      0.5 * accel_mag * std::pow(_lookahead_time, 2)
      + velocity_mag * _lookahead_time
  );
  // LOG(INFO) << "Calculated look_ahead_distance: " << look_ahead_distance;
  look_ahead_distance = std::min(
      std::max(look_ahead_distance,
               _lookahead_distance_min
      ),
      _lookahead_distance_max
  );
  // LOG(INFO) << "Final look_ahead_distance: " << look_ahead_distance;
  return look_ahead_distance;
}


/* Returns the updated goal-state of the ego-vehicle.
 *
 * @param    ego_state  Current state of the ego-vehicle to update.
 * @param    map        Map containing the goal-state waypoints.
 * @returns  goal       Goal-state updated w.r.t. current state.
 */
State BehaviorPlannerFSM::get_goal(
    const State& ego_state,
    SharedPtr<carla::client::Map> map
) {
  // Get look-ahead distance based on ego-vehicle speed
  auto look_ahead_distance = get_look_ahead_distance(ego_state);
  // Nearest waypoint situated in the centre of a driving lane
  bool is_goal_in_junction{false};
  auto goal_wp = get_closest_waypoint_goal(
      ego_state,
      map,
      look_ahead_distance,
      is_goal_in_junction
  );
  // LOG(INFO) << "Is the FINAL goal on a junction: " << is_goal_in_junction;
  string tl_state = "none";
  State goal = state_transition(
      ego_state,
      goal_wp,
      is_goal_in_junction,
      tl_state
  );
  return goal;
}