/* ------------------------------------------------------------------------------
 * Lesson "4.1: Behavior Planning"
 * Authors     : Benjamin Ulmer and Tobias Roth of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the cost functions.
 * ----------------------------------------------------------------------------
 */

#ifndef 3_COSTS_H_
#define 3_COSTS_H_


#include "3_vehicle.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>


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
);


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
    const Vehicle &vehicle,  
    const std::vector<Vehicle>& trajectory,  
    const std::map<int, std::vector<Vehicle>>& predictions, 
    std::map<std::string, float>& data
);


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
);


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
);


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
);


#endif  // 3_COSTS_H