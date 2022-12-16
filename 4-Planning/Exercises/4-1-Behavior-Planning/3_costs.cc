/* ------------------------------------------------------------------------------
 * Lesson "4.1: Behavior Planning"
 * Authors     : Benjamin Ulmer and Tobias Roth of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the cost functions used in the trajectory
 *                       planner. Provided here are two possible cost functions
 *                       to use. The weighted cost over all selected cost
 *                       functions is computed in `calculate_cost`.
 * ----------------------------------------------------------------------------
 */


#include "3_costs.h"

/*** Setting weight values for cost functions ***/
// TODO: Modify these values to something reasonable
const float REACH_GOAL = std::pow(10, 6);
const float EFFICIENCY = std::pow(10, 5);


/* Computes the sum of the weighted cost functions for the current trajectory. 
 *
 * @param    vehicle      `Vehicle` state object to evaluate.
 * @param    predictions  Surrounding vehicle trajectories.
 * @param    trajectory   Planned manoeuvre to compute the cost of. 
 * @returns  cost         Summed weighted cost for the current trajectory.
 */
float calculate_cost(
    const Vehicle& vehicle, 
    const std::map<int, std::vector<Vehicle>>& predictions, 
    const std::vector<Vehicle>& trajectory
) {
  // Get the current state info for this state vector
  std::map<std::string, float> trajectory_data = get_helper_data(
      vehicle,
      trajectory,
      predictions
  );
  float cost = 0.0;
  // CANDO: Add additional cost functions here.
  std::vector<std::function<float(
      const Vehicle&, 
      const std::vector<Vehicle>&, 
      const std::map<int, std::vector<Vehicle>>&, 
      std::map<std::string, float>&
  )>> cf_list = {goal_distance_cost, inefficiency_cost};
  // CANDO: Add additional cost function weights here.
  std::vector<float> weight_list = {REACH_GOAL, EFFICIENCY};
  // Compute the weighted sum of the cost values   
  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i] * cf_list[i](vehicle, 
                                                 trajectory, 
                                                 predictions, 
                                                 trajectory_data
    );
    cost += new_cost;
  }
  return cost;
}


/* Computes the goal-distance cost given any lane changes.
 *
 * Here the cost increases when: 
 *     (a) Lateral distance of intended / final lane increases w.r.t. goal lane;
 *     (b) Longitudinal distance to goal marker decreases.
 * 
 * See Exercise 4.1.1 (`1_distance_cost.cc`) for more information.
 * 
 * @param    vehicle      `Vehicle` state object to evaluate.
 * @param    trajectory   Planned manoeuvre to compute the cost of.
 * @param    predictions  Surrounding vehicle trajectories. 
 * @param    data         Environment state with the distance to goal.
 * @returns  cost         Computed goal-distance cost value. 
 */
float goal_distance_cost(
    const Vehicle& vehicle, 
    const std::vector<Vehicle>& trajectory, 
    const std::map<int, std::vector<Vehicle>>& predictions, 
    std::map<std::string, float>& data
) {
  float cost;
  float distance = data["distance_to_goal"];
  if (distance > 0) {
    float delta_d = 2.0 * vehicle.goal_lane;
    delta_d = delta_d - data["intended_lane"] - data["final_lane"];
    cost = 1 - 2 * exp(-(abs(delta_d) / distance));
  } 
  else {
    cost = 1;
  }
  return cost;
}


/* Computes the inefficiency cost of the goal speed given lane changes.
 *
 * Here the cost increases for trajectories with:
 *     (a) Intended lane traffic moving slower than vehicle target speed;
 *     (b) Final lane traffic moving slower than vehicle target speed. 
 *
 * See Exercise 1.4.2 (`2_inefficiency_cost.cc`) for more information.
 * 
 * @param    vehicle      `Vehicle` state object to evaluate.
 * @param    trajectory   Planned manoeuvre to compute the cost of.
 * @param    predictions  Surrounding vehicle trajectories.
 * @param    data         Environment state with the intended / final lane.
 * @returns  cost         Computed inefficiency cost value.
 */
float inefficiency_cost(
    const Vehicle& vehicle, 
    const std::vector<Vehicle>& trajectory, 
    const std::map<int, std::vector<Vehicle>>& predictions, 
    std::map<std::string, float>& data
) {
  float proposed_speed_intended = lane_speed(
      predictions, 
      data["intended_lane"]
  );
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = vehicle.target_speed;
  }
  float proposed_speed_final = lane_speed(
      predictions, 
      data["final_lane"]
  );
  if (proposed_speed_final < 0) {
    proposed_speed_final = vehicle.target_speed;
  }
  float delta_d = 2.0 * vehicle.target_speed;
  delta_d = delta_d - proposed_speed_intended - proposed_speed_final;  
  float cost = delta_d / vehicle.target_speed;
  return cost;
}


/* Get the current lane speed given by a non-ego vehicle.
 *
 * Here we assume that the 'lane speed' is equal to the current velocity
 * of any non-ego vehicle occupying the respective lane. Also assumed is 
 * that all non-ego vehicles in any given lane move at the same speed.
 * Therefore, only one vehicle from a given lane needs to be evaluated to
 * obtain the current velocity ("speed") of the traffic in that lane. 
 *
 * @param    predictions
 * @param    lane
 * @returns  Velocity of the vehicle found in lane, else -1 if none found.
 */
float lane_speed(
    const std::map<int, std::vector<Vehicle>>& predictions, 
    int lane
) {
  std::map<int, std::vector<Vehicle>>::const_iterator it;
  for (it = predictions.begin(); it != predictions.end(); ++it) {
    int key = it->first;
    Vehicle vehicle = it->second[0];
    if ((vehicle.lane == lane) 
        && (key != -1)
    ) {
      return vehicle.v;
    }
  }
  // Found no vehicle in the lane
  return -1.0;
}


/* Generates the helper data used in the cost function evaluation.
 *
 * Simulates the next time-step state, i.e., the intended / final lanes
 * for a given vehicle state (e.g., "PLCL" sets the intended lane to the
 * next lane after the left move is executed).
 *
 * @param    vehicle          `Vehicle` state object to evaluate.
 * @param    trajectory       Planned manoeuvre to get the next-state info of.
 * @param    predictions      Surrounding vehicle trajectories.
 * @returns  trajectory_data  Contains the state info for this time-step.
 *
 */
std::map<std::string, float> get_helper_data(
    const Vehicle& vehicle, 
    const std::vector<Vehicle>& trajectory, 
    const std::map<int, std::vector<Vehicle>>& predictions
) {
  std::map<std::string, float> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  float intended_lane;
  // Set the intended lane +/- 1 according to the planned turn
  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } 
  else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane - 1;
  }
  else {
    intended_lane = trajectory_last.lane;
  }
  // Compute updated distance to goal 
  float distance_to_goal = vehicle.goal_s - trajectory_last.s;
  float final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  // Final lane here differentiates between planned and executed lane changes
  // i.e., difference between "PLCL" and "LCL" used in the cost functions
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;
  return trajectory_data;
}