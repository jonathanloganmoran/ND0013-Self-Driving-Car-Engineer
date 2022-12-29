/* ------------------------------------------------------------------------------
 * Project "4.1: Motion Planning and Decision Making for Autonomous Vehicles"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the velocity profile generator.
 * ----------------------------------------------------------------------------
 */

#include "velocity_profile_generator.h"


VelocityProfileGenerator::VelocityProfileGenerator() {}
VelocityProfileGenerator::~VelocityProfileGenerator() {}


/* Initialises the velocity profile generator with the given state variables.
 *
 * The maximum permitted acceleration (`a_max`) is set w.r.t. the comfort
 * and vehicle / feasibility constraints.
 * 
 * NOTE: The parameters marked `TODO` have not been clearly defined.
 *
 * @param  time_gap     Time-gap (s) to use with the velocity profile. TODO.
 * @param  a_max        Maximum permitted acceleration (m/s^2).
 * @param  slow_speed   Speed to set for slow manoeuvre (m/s). TODO.
 */
void VelocityProfileGenerator::setup(
    const double& time_gap,
    const double& a_max,
    const double& slow_speed
) {
  _time_gap = time_gap;
  _a_max = a_max;
  _slow_speed = slow_speed;
};


/* Returns the trajectory generated for the given state variables.
 *
 * @param    spiral           Path containing the waypoints to profile.
 * @param    desired_speed    Desired velocity (m/s) to maintain.
 * @param    ego_state        Ego-vehicle starting state.
 * @param    lead_car_state   Lead-vehicle state to match.
 * @param    maneuver         Requested vehicle state to create trajectory for.
 * @returns  trajectory       Trajectory computed w.r.t. the given state.
 */
std::vector<TrajectoryPoint> VelocityProfileGenerator::generate_trajectory(
    const std::vector<PathPoint>& spiral, 
    const double& desired_speed,
    const State& ego_state, 
    const State& lead_car_state,
    const Maneuver& maneuver
) const {
  // LOG(INFO) << "Lead car x: " << lead_car_state.location.x;
  std::vector<TrajectoryPoint> trajectory;
  double start_speed = utils::magnitude(ego_state.velocity);
  // LOG(INFO) << "Start Speed (m/s): " << start_speed;
  // LOG(INFO) << "Desired Speed (m/s): " << desired_speed;
  // Generate a trapezoidal trajectory to decelerate to stop.
  if (maneuver == DECEL_TO_STOP) {
    // LOG(INFO) << "Generating velocity trajectory for DECEL_TO_STOP";
    trajectory = decelerate_trajectory(
        spiral, 
        start_speed
    );
  }
  // If we need to follow the lead vehicle, make sure we decelerate to its speed
  // by the time we reach the time gap point.
  else if (maneuver == FOLLOW_VEHICLE) {
    // LOG(INFO) << "Generating velocity trajectory for FOLLOW_VEHICLE";
    trajectory = follow_trajectory(
        spiral, 
        start_speed, 
        desired_speed, 
        lead_car_state
    );
  }
  // Otherwise, compute the trajectory to reach our desired speed.
  else {
    // LOG(INFO) << "Generating velocity trajectory for NOMINAL TRAVEL";
    trajectory = nominal_trajectory(
        spiral, 
        start_speed, 
        desired_speed
    );
  }
  // Interpolate between the zeroth state and the first state.
  // This prevents the controller from getting stuck at the zeroth state.
  if (trajectory.size() > 1) {
    TrajectoryPoint interpolated_state;
    interpolated_state.path_point.x = (
      (trajectory[1].path_point.x - trajectory[0].path_point.x) * 0.1 
      + trajectory[0].path_point.x
    );
    interpolated_state.path_point.y = (
      (trajectory[1].path_point.y - trajectory[0].path_point.y) * 0.1
      + trajectory[0].path_point.y
    );
    interpolated_state.path_point.z = (
      (trajectory[1].path_point.z - trajectory[0].path_point.z) * 0.1 
      + trajectory[0].path_point.z
    );
    interpolated_state.v = (
      (trajectory[1].v - trajectory[0].v) * 0.1 
      + trajectory[0].v
    );
    trajectory[0] = interpolated_state;
  }
  return trajectory;
}


/* Returns the velocity trajectory for deceleration to a full stop.
 *
 * The trajectory for the `DECEL_TO_STOP` state is computed w.r.t. the
 * given path and initial vehicle speed. A comfortable deceleration rate is
 * used to determine a safe distance needed for the vehicle to travel in
 * order to arrive at a complete stop. 
 *
 * @param    spiral         Path containing the waypoints to profile.
 * @param    start_speed    Initial ego-vehicle speed (m/s).
 * @returns  trajectory     Deceleration-to-stop trajectory.
 */
std::vector<TrajectoryPoint> VelocityProfileGenerator::decelerate_trajectory(
    const std::vector<PathPoint>& spiral, 
    const double& start_speed
) const {
  std::vector<TrajectoryPoint> trajectory;
  // Using d = (v_f^2 - v_i^2) / (2 * a)
  auto decel_distance = calc_distance(start_speed, _slow_speed, -_a_max);
  auto brake_distance = calc_distance(_slow_speed, 0, -_a_max);
  auto path_length{0.0};
  auto stop_index{spiral.size() - 1};
  for (size_t i = 0; i < stop_index; ++i) {
    path_length += utils::distance(spiral[i + 1], spiral[i]);
  }
  /* If the brake distance exceeds the length of the path, then we cannot
  perform a smooth deceleration and require a harder deceleration.  Build the path
  up in reverse to ensure we reach zero speed at the required time.
  */
  if ((brake_distance + decel_distance) > path_length) {
    std::vector<double> speeds;
    auto vf{0.0};
    // Let's add the last point, i.e at the stopping line we should have speed
    // 0.0.
    auto it = speeds.begin();
    speeds.insert(it, 0.0);
    // Let's now go backwards until we get to the very beginning of the path
    for (int i = stop_index - 1; i >= 0; --i) {
      auto dist = utils::distance(spiral[i + 1], spiral[i]);
      auto vi = calc_final_speed(vf, -_a_max, dist);
      if (vi > start_speed) {
        vi = start_speed;
      }
      // Let's add it
      auto it = speeds.begin();
      speeds.insert(it, vi);
      vf = vi;
    }
    // At this point we have all the speeds. Now we need to create the
    // trajectory
    double time_step{0.0};
    double time{0.0};
    for (size_t i = 0; i < speeds.size() - 1; ++i) {
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = speeds[i];
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      time_step = std::fabs(speeds[i] - speeds[i + 1]) / _a_max;  // Doubt!
      time += time_step;
    }
    // We still need to add the last one
    auto i = spiral.size() - 1;
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = speeds[i];
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);

    // If the brake distance DOES NOT exceed the length of the path
  } 
  else {
    auto brake_index{stop_index};
    auto temp_dist{0.0};
    // Compute the index at which to start braking down to zero.
    while ((brake_index > 0) 
           && (temp_dist < brake_distance)
    ) {
      temp_dist += (
          utils::distance(spiral[brake_index], 
                          spiral[brake_index - 1])
      );
      --brake_index;
    }
    // Compute the index to stop decelerating to the slow speed.
    uint decel_index{0};
    temp_dist = 0.0;
    while ((decel_index < brake_index) 
           && (temp_dist < decel_distance)
    ) {
      temp_dist += (
          utils::distance(spiral[decel_index + 1], 
                          spiral[decel_index])
      );
      ++decel_index;
    }
    // At this point we have all the speeds. Now we need to create the
    // trajectory
    double time_step{0.0};
    double time{0.0};
    auto vi{start_speed};
    for (size_t i = 0; i < decel_index; ++i) {
      auto dist = utils::distance(spiral[i + 1], spiral[i]);
      auto vf = calc_final_speed(vi, -_a_max, dist);
      if (vf < _slow_speed) {
        vf = _slow_speed;
      }
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = vi;
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      time_step = std::fabs(vf - vi) / _a_max;
      time += time_step;
      vi = vf;
    }
    for (size_t i = decel_index; i < brake_index; ++i) {
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = vi;
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      auto dist = utils::distance(spiral[i + 1], spiral[i]);  // ??
      if (dist > DBL_EPSILON) {
        time_step = vi / dist;
      }
      else {
        time_step = 0.00;
      }
      time += time_step;
    }
    for (size_t i = brake_index; i < stop_index; ++i) {
      auto dist = utils::distance(spiral[i + 1], spiral[i]);
      auto vf = calc_final_speed(vi, -_a_max, dist);
      TrajectoryPoint traj_point;
      traj_point.path_point = spiral[i];
      traj_point.v = vi;
      traj_point.relative_time = time;
      trajectory.push_back(traj_point);
      time_step = std::fabs(vf - vi) / _a_max;
      time += time_step;
      vi = vf;
    }
    // Now we just need to add the last point.
    auto i = stop_index;
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = 0.0;
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);
  }
  return trajectory;
}


/* Returns the velocity trajectory for following a lead vehicle.
 *
 * @param    spiral       Path containing the waypoints to profile.
 * @param    start_speed  Initial ego-vehicle speed (m/s).
 * @returns  trajectory   Lead-vehicle follow trajectory.
 */
std::vector<TrajectoryPoint> VelocityProfileGenerator::follow_trajectory(
    const std::vector<PathPoint>& spiral, 
    const double& start_speed,
    const double& desired_speed, 
    const State& lead_car_state
) const {
  std::vector<TrajectoryPoint> trajectory;
  return trajectory;
}


/* Returns the velocity trajectory for nominal speed tracking.
 *
 * Here, the nominal speed tracking trajectory is computed for either the 
 * Lane Follow or Cruise Control vehicle states.
 *
 * @param    spiral       Path containing the waypoints to profile.
 * @param    start_speed  Initial ego-vehicle speed (m/s).
 * @returns  trajectory   Nominal speed trajectory.
 */
std::vector<TrajectoryPoint> VelocityProfileGenerator::nominal_trajectory(
    const std::vector<PathPoint>& spiral, 
    const double& start_speed,
    double const& desired_speed
) const {
  std::vector<TrajectoryPoint> trajectory;
  double accel_distance;
  // LOG(INFO) << "MAX_ACCEL: " << _a_max;
  if (desired_speed < start_speed) {
    // LOG(INFO) << "decelerate";
    accel_distance = calc_distance(start_speed, desired_speed, -_a_max);
  } 
  else {
    // LOG(INFO) << "accelerate";
    accel_distance = calc_distance(start_speed, desired_speed, _a_max);
  }
  size_t ramp_end_index{0};
  double distance{0.0};
  while (ramp_end_index < (spiral.size() - 1) 
         && (distance < accel_distance)
  ) {
    distance += (
        utils::distance(spiral[ramp_end_index], 
                        spiral[ramp_end_index + 1])
    );
    ramp_end_index += 1;
  }
  // LOG(INFO) << "ramp_end_index:" << ramp_end_index;
  double time_step{0.0};
  double time{0.0};
  double vi{start_speed};
  for (size_t i = 0; i < ramp_end_index; ++i) {
    auto dist = utils::distance(spiral[i], spiral[i + 1]);
    double vf;
    if (desired_speed < start_speed) {
      vf = calc_final_speed(vi, -_a_max, dist);
      if (vf < desired_speed) {
        vf = desired_speed;
      }
    } 
    else {
      vf = calc_final_speed(vi, _a_max, dist);
      if (vf > desired_speed) {
        vf = desired_speed;
      }
    }
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = vi;
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);
    // LOG(INFO) << i << "- x: " << traj_point.path_point.x
    //          << ", y: " << traj_point.path_point.y
    //          << ", th: " << traj_point.path_point.theta
    //          << ", v: " << traj_point.v << ", t: " <<
    //          traj_point.relative_time;
    time_step = std::fabs(vf - vi) / _a_max;
    time += time_step;
    vi = vf;
  }
  for (size_t i = ramp_end_index; i < spiral.size() - 1; ++i) {
    TrajectoryPoint traj_point;
    traj_point.path_point = spiral[i];
    traj_point.v = desired_speed;
    traj_point.relative_time = time;
    trajectory.push_back(traj_point);
    auto dist = utils::distance(spiral[i], spiral[i + 1]);
    // This should never happen in a "nominal_trajectory", but it's a sanity
    // check
    if (std::abs(desired_speed) < DBL_EPSILON) {
      time_step = 0.0;
    } 
    else {
      time_step = dist / desired_speed;
    }
    time += time_step;
    // LOG(INFO) << i << "- x: " << traj_point.path_point.x
    //          << ", y: " << traj_point.path_point.y
    //          << ", th: " << traj_point.path_point.theta
    //          << ", v: " << traj_point.v << ", t: " <<
    //          traj_point.relative_time;
  }
  // Add last point
  auto i = spiral.size() - 1;
  TrajectoryPoint traj_point;
  traj_point.path_point = spiral[i];
  traj_point.v = desired_speed;
  traj_point.relative_time = time;
  trajectory.push_back(traj_point);
  // LOG(INFO) << i << "- x: " << traj_point.path_point.x
  //          << ", y: " << traj_point.path_point.y
  //          << ", th: " << traj_point.path_point.theta
  //          << ", v: " << traj_point.v << ", t: " << traj_point.relative_time;
  // LOG(INFO) << "Trajectory Generated";
  return trajectory;
}


/* Computes the travel distance needed for given acceleration / deceleration.
 *
 * The distance required to complete the manoeuvre is given by the distance
 * equation of the 1D rectilinear motion, i.e.,
 *   $d = (v_{f}^{2} + v_{i}^{2}) / (2 * a)$.
 * 
 * @param    v_i  Initial velocity (m/s) of the vehicle to profile.
 * @param    v_f  Final desired velocity (m/s) to profile.
 * @param    a    Desired acceleration (m/s^2) to profile.
 * @returns  d    Distance required to complete the manoeuvre (m).
 */
double VelocityProfileGenerator::calc_distance(
    const double& v_i,
    const double& v_f,
    const double& a
) const {
  double d{0.0};
  if (std::abs(a) < DBL_EPSILON) {
    d = std::numeric_limits<double>::infinity();
  } 
  else {
    // TODO-calc distance: use one of the common rectilinear accelerated
    // equations of motion to calculate the distance traveled while going from
    // v_i (initial velocity) to v_f (final velocity) at a constant
    // acceleration/deceleration "a". HINT look at the description of this
    // function. Make sure you handle div by 0
    d = 0;  // <- Update
  }
  return d;
}

/* Computes the final velocity of the trajectory given by the state variables.
 *
 * The final velocity is given by the 1D rectilinear motion equation, i.e.,
 *   $v_{f} = \sqrt{v_{i}^{2} + 2 * a * d}$,
 * for the input state variables of the vehicle.
 * 
 * NOTE: If the discriminant inside the radical is negative, a final velocity
 * of `0.0` is returned. 
 *
 * @param    v_i    Initial velocity (m/s) of the vehicle to profile.
 * @param    a      Acceleration of the vehicle during the trajectory.
 * @param    d      Distance travelled (m) by the vheicle during the manoeuvre.
 * @returns  v_f    Final velocity (m/s) of vehicle after complete trajectory.
 */
double VelocityProfileGenerator::calc_final_speed(
    const double& v_i,
    const double& a,
    const double& d
) const {
  double v_f{0.0};
  // TODO-calc final speed: Calculate the final distance. HINT: look at the
  // description of this function. Make sure you handle negative discriminant
  // and make v_f = 0 in that case. If the discriminant is inf or nan return
  // infinity
  double disc = 0;  // <- Fix this
  if (disc <= 0.0) {
    v_f = 0.0;
  } 
  else if (disc == std::numeric_limits<double>::infinity() 
           || std::isnan(disc)
  ) {
    v_f = std::numeric_limits<double>::infinity();
  } 
  else {
    v_f = std::sqrt(disc);
  }
  //   std::cout << "v_i, a, d: " << v_i << ", " << a << ", " << d
  //             << ",  v_f: " << v_f << std::endl;
  return v_f;
}
