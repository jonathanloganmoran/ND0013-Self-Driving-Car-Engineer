/* ------------------------------------------------------------------------------
 * Lesson "4.1: Behavior Planning"
 * Authors     : Benjamin Ulmer and Tobias Roth of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the `Vehicle` class and trajectory planner.
 *                       Computes the `trajectory`, i.e., a vector containing
 *                       the previous and updated `Vehicle` states after a
 *                       manoeuvre has been made (e.g., "KL" — keep lane).
 * ----------------------------------------------------------------------------
 */


#include "3_vehicle.h"


// Initialises the `Vehicle` class constructor
Vehicle::Vehicle(){}


// Implements the `Vehicle` class constructor
Vehicle::Vehicle(
    int lane, 
    float s, 
    float v, 
    float a, 
    std::string state
) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
  max_acceleration = -1;
}

// Initialises the `Vehicle` class destructor
Vehicle::~Vehicle() {}


/* Selects the most-optimal (lowest cost) trajectory from the FSM.
 *
 * The set of input `predictions` map vehicle ids to their pair of
 * `Vehicle` states, one from the previous time-step and one from the current. 
 * Each vehicle therefore has a set of two `Vehicle` states to consider.
 * 
 * @param    predictions  `Vehicle` states, each vehicle `id` has one pair. 
 * @returns  Best (lowest cost) trajectory found in the next ego-vehicle state.
 * */
std::vector<Vehicle> Vehicle::choose_next_state(
    std::map<int, std::vector<Vehicle>>& predictions
) {
  // Get from the FSM the next possible states given the current
  std::vector<std::string> states = successor_states();
  std::vector<float> costs;
  for (std::string state : states) {
    std::vector<Vehicle> trajectory = generate_trajectory(
        state,
        predictions
    );
    if (trajectory) {
      // Calculate the cost of this trajectory
      float cost = calculate_cost(
          vehicle
          predictions,
          trajectory
      ); 
      costs.append(cost);
    }
  }
  // Return the minimum cost value
  return *min_element(costs.begin(), costs.end());
}


/* Evaluates the current state in the FSM to obtain next possible states.
 *
 * Here the finite-state machine (FSM) resembles the course literature,
 * but with the assumption that lane changes happen instantaneously, 
 * i.e., states "LCL" and "LCR" can transition to "KL" (keep lane).
 * 
 * Note: the result of `string::compare()` is `0` if the two strings are equal.
 */
std::vector<std::string> Vehicle::successor_states() {
  std::vector<std::string> states;
  // Initialise the next-state after lane change is performed 
  states.push_back("KL");
  std::string state = this->state;
  if (state.compare("KL") == 0) {
    // Ego-vehicle keeping lane can choose to prepare a lane change
    // i.e., can prepare move to either left or right lane
    states.push_back("PLCL");
    states.push_back("PLCR");
  } 
  else if (state.compare("PLCL") == 0) {
    // Ego-vehicle has intention of preparing lane change left
    if (lane != lanes_available - 1) {
      // Has room to move left, 
      // Can proceed with lane change or remain in state
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } 
  else if (state.compare("PLCR") == 0) {
    // Ego-vehicle has intention of preparing lane change right
    if (!lane) {
      // Has room to move right (i.e., not at lane 0),
      // Can proceed with lane change or remain in state
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }
  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}


/* Evaluates the desired vehicle manoeuvre / state transition.
 *
 * The given `state` string indicates the desired ego-vehicle manoeuvre
 * to evaluate, i.e., a `state` of "CS" will return the current `Vehicle`
 * state and the updated state where the vehicle remains in the current
 * lane with a "constant" (unchanged) velocity and an acceleration of zero. 
 * 
 * The `trajectory` is a vector of the previous and the updated `Vehicle`
 * state objects.
 *
 * @param    state        Desired vehicle manoeuvre / state transition.
 * @param    predictions  Next-state trajectories of vehicles on road.
 * @returns  trajectory   Current and next time-step `Vehicle` state objects. 
 */
std::vector<Vehicle> Vehicle::generate_trajectory(
    std::string state,
    std::map<int, std::vector<Vehicle>>& predictions
) {
  std::vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }
  return trajectory;
}


/* Computes the 1D kinematics for the next time-step for a given lane.
 *
 * The new velocity and acceleration are computed w.r.t. the movement of
 * the other vehicles surrounding the ego-vehicle in either front or rear,
 * given the acceleration, displacement and velocity contraints.  
 *
 * @param    predictions  Next-state trajectories of vehicles on road.
 * @param    lane         Desired lane to move to in next time-step.
 * @returns  Set of 1D kinematics `{position, velocity, acceleration}`.
 */
std::vector<float> Vehicle::get_kinematics(
    std::map<int, std::vector<Vehicle>>& predictions, 
    int lane
) {
  // Compute the maximum theoretical next-state velocity
  // using 1D kinematics equation for velocity where $t=1$ 
  float max_velocity_accel_limit = this->v + this->max_acceleration;
  float new_position;
  float new_velocity;
  float new_accel;
  Vehicle vehicle_ahead;    // To be updated with vehicle found in front of ego
  Vehicle vehicle_behind;   // To be updated with vehicle found in rear of ego
  if (
    get_vehicle_ahead(predictions, lane, vehicle_ahead)
  ) {
    if (
      get_vehicle_behind(predictions, lane, vehicle_behind)
    ) {
      // If ego-vehicle is sandwiched between cars (in front and in rear)
      // then set updated ego-vehicle velocity to match lead vehicle velocity,
      // irrespective of ego-vehicle's preferred buffer / following distance.
      new_velocity = vehicle_ahead.v;
    } 
    else {
      // No vehicles in front or rear of ego
      // Compute new velocity w.r.t. preferred buffer / following distance
      // using 1D kinematics equation for displacement where $t=1$
      float max_velocity_in_front = (
          (vehicle_ahead.s - this->s - this->preferred_buffer) 
          + vehicle_ahead.v 
          - 0.5 * (this->a)
      );
      // Clip the new velocity to the minimum feasible velocity
      // i.e., the lowest between acceler
      new_velocity = std::min(
          std::min(max_velocity_in_front, 
                   max_velocity_accel_limit
          ), 
          this->target_speed
      );
    }
  } 
  else {
    new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
  }
  // Compute new acceleration w.r.t. previous and new velocity
  // using the 1D kinematics equation for velocity where $t=1$
  // i.e., $a = (v_1 - v_0) / t$
  new_accel = new_velocity - this->v;
  // Compute new position w.r.t. prev. position and new acceleration / velocity
  // using the 1D kinematics equation for displacement where $t=1$
  new_position = this->s + new_velocity + 0.5 * new_accel;
  // Return the updated 1D vehicle kinematics
  return {new_position, new_velocity, new_accel};
}


/* Returns a constant speed trajectory.
 *
 * A constant-speed trajectory updates the vehicle to the next position
 * given its velocity. The other state variables (lane, state) will remain
 * the same as before. The acceleration (if any) will be set to zero. 
 *
 * @returns  trajectory   Current and next-state transition after lane change. 
 */
std::vector<Vehicle> Vehicle::constant_speed_trajectory() {
  float next_pos = position_at(1);
  std::vector<Vehicle> trajectory = {
      Vehicle(this->lane, 
              this->s, 
              this->v, 
              this->a, 
              this->state
      ), 
      Vehicle(this->lane, 
              next_pos, 
              this->v, 
              0,
              this->state
      )
  };
  return trajectory;
}


/* Returns the trajectory for a lane keep.
 *
 * @param    predictions  Set of estimated `Vehicle` trajectories.
 * @returns  trajectory   Trajectory of the vehicle after a lane keep.
 */
std::vector<Vehicle> Vehicle::keep_lane_trajectory(
    std::map<int, std::vector<Vehicle>>& predictions
) {
  // Generate a keep lane trajectory.
  vector<Vehicle> trajectory = {
      Vehicle(lane, 
              this->s, 
              this->v, 
              this->a, 
              state
      )
  };
  vector<float> kinematics = get_kinematics(
      predictions, 
      this->lane
  );
  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];
  trajectory.push_back(
      Vehicle(this->lane, 
              new_s, 
              new_v, 
              new_a, 
              "KL"
      )
  );
  return trajectory;
}


/* Returns the best trajectory for the desired lange change.
 *
 * If no trajectories exist for the given state / predictions, then an
 * empty vector is returned. The trajectory matching the desired
 * lane change with the best kinematics is given. Here, we define the
 * "best" kinematics to be the lane with the lowest change in velocity. 
 *
 * @param    state        Direction to move (i.e., left or right).
 * @param    predictions  Set of trajectories to execute lane change for.
 * @returns  trajectory   Trajectory with the best kinematics and state.
 */
std::vector<Vehicle> Vehicle::prep_lane_change_trajectory(
    std::string state,
    std::map<int, std::vector<Vehicle>>& predictions
) {
  // Generate a trajectory preparing for a lane change.
  float new_s;
  float new_v;
  float new_a;
  Vehicle vehicle_behind;
  int new_lane = this->lane + lane_direction[state];
  std::vector<Vehicle> trajectory = {
      Vehicle(this->lane, 
              this->s, 
              this->v, 
              this->a,
              this->state
      )
  };
  std::vector<float> curr_lane_new_kinematics = get_kinematics(
      predictions,
      this->lane
  );
  if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];    
  } else {
    std::vector<float> best_kinematics;
    std::vector<float> next_lane_new_kinematics = get_kinematics(
        predictions, 
        new_lane
    );
    // Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } 
    else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }
  trajectory.push_back(
    Vehicle(this->lane, 
            new_s, 
            new_v, 
            new_a, 
            state
    )
  );
  return trajectory;
}


/* Generates a `Vehicle` trajectory which includes a lane change (if possible). 
 *
 * @param    state        Direction to move in (i.e., left or right).   
 * @param    predictions  Vector of predicted `Vehicle` trajectories. 
 * @returns  trajectory   A `Vehicle` trajectory (empty if not possible).
 */
std::vector<Vehicle> Vehicle::lane_change_trajectory(
    std::string state,
    std::map<int, std::vector<Vehicle>>& predictions
) {
  // Get the lane id of the lane to change to
  int new_lane = this->lane + lane_direction[state];
  // Generate a new trajectory
  std::vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Checking if lane change is possible, i.e., if another vehicle
  // does not occupy the desired spot
  std::map<int, std::vector<Vehicle>>::iterator it;
  for (it = predictions.begin(); it != predictions.end(); ++it) {
    next_lane_vehicle = it->second[0];
    if ((next_lane_vehicle.s == this->s) 
        && (next_lane_vehicle.lane == new_lane)
    ) {
      // Lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  // Not occupied, get the next vehicle state given 1D kinematics
  // Add the current vehicle state to the trajectory vector 
  trajectory.push_back(
      Vehicle(this->lane,
              this->s, 
              this->v, 
              this->a,
              this->state
      )
  );
  // Compute the new position, velocity and acceleration
  std::vector<float> kinematics = get_kinematics(
      predictions, 
      new_lane
  );
  // Add the updated vehicle state post-manoeuvre to the trajectory vector 
  trajectory.push_back(
      Vehicle(new_lane, 
              kinematics[0], 
              kinematics[1],
              kinematics[2], 
              state
      )
  );
  return trajectory;
}


/* Updates the vehicle position to the next time-step.
 *
 * @param  dt   Elapsed time to calculate the change in position w.r.t.
 */
void Vehicle::increment(
    int dt=1
) {
  this->s = position_at(dt);
}


/* Computes the new position of the vehicle after `t` time-steps have passed.
 *
 * @param    t  Elapsed time to consider w.r.t. vehicle velocity.
 * @returns  New position of the vehicle w.r.t. 1D kinematics.
 */
float Vehicle::position_at(
    int t
) {
  return this->s + this->v * t + 0.5 * this->a * t * t;
}


/* Checks if a vehicle exists behind the given vehicle.
 *
 * @param    predictions    Vector of vehicle trajectories to search.
 * @param    lane           Lane to check for vehicle presence in rear.
 * @param    rVehicle       Uninitialised `Vehicle` to return if one is found.
 * @returns  found_vehicle  Boolean indicating whether a vehicle was found. 
 * */
bool Vehicle::get_vehicle_behind(
    std::map<int, std::vector<Vehicle>>& predictions, 
    int lane, 
    Vehicle& rVehicle
) {
  // Returns a true if a vehicle is found behind the current vehicle, false 
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  std::map<int, std::vector<Vehicle>>::iterator it;
  for (it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if ((temp_vehicle.lane == this->lane) 
        && (temp_vehicle.s < this->s) 
        && (temp_vehicle.s > max_s)
    ) {
      max_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}


/* Checks if a vehicle exists in front of the given vehicle.
 * 
 * Updates the `rVehicle` to the vehicle that is both:
 *     (a) in front of the `rVehicle` in the given `lane`, and
 *     (b) the closest to the `rVehicle` from the given `lane`,
 * i.e.,  any other vehicles in the same lane but farther ahead will
 * not be used to update the `rVehicle`.
 *
 * @param  predictions      Vector of vehicle trajectories to search.
 * @param  lane             Lane to check for vehicle presence in.
 * @param  rVehicle         Uninitialised `Vehicle` to return if one is found.
 * @param  found_vehicle    Boolean indicating whether a vehicle was found.
 * */
bool Vehicle::get_vehicle_ahead(
    std::map<int, std::vector<Vehicle>>& predictions, 
    int lane,
    Vehicle& rVehicle
) {
  int min_s = this->goal_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  std::map<int, std::vector<Vehicle>>::iterator it;
  for (it = predictions.begin(); it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    if ((temp_vehicle.lane == this->lane) 
        && (temp_vehicle.s > this->s) 
        && (temp_vehicle.s < min_s)
    ) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  return found_vehicle;
}


/* Predicts the trajectories of the non-ego vehicles to the time horizon.
 * Returns the predicted non-ego vehicle trajectories for `horizon` number
 * of time-steps into the future. 
 * 
 * @param    horizon      Number of time-steps in the future to predict for.
 * @returns  predictions  Non-ego vehicle trajectories `horizon` steps in future.
 */
std::vector<Vehicle> Vehicle::generate_predictions(
    int horizon
) {
  // Generates predictions for non-ego vehicles to be used in trajectory 
  //   generation for the ego vehicle.
  std::vector<Vehicle> predictions;
  for(int i = 0; i < horizon; ++i) {
    float next_s = position_at(i);
    float next_v = 0;
    if (i < horizon - 1) {
      next_v = position_at(i + 1) - s;
    }
    predictions.push_back(
        Vehicle(this->lane, 
                next_s, 
                next_v, 
                0
        )
    );
  }
  return predictions;
}


/* Updates the ego-vehicle state and kinematics using the given trajectory.
 * 
 * @param  trajectory   Vector of trajectories with the last state to use.
 */
void Vehicle::realize_next_state(
    std::vector<Vehicle>& trajectory
) {
  // Sets state and kinematics for ego vehicle using the last state of the trajectory.
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
  this->a = next_state.a;
}


/* Sets-up and initialises the starting state variables.
 *
 * @param  road_data  Vector containing the state informmation to use.
 */
void Vehicle::configure(
    std::vector<int>& road_data
) {
  // Called by simulator before simulation begins. Sets various parameters
  // which will impact the ego vehicle.
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
}