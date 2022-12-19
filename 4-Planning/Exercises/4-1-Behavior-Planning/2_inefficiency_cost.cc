/* ------------------------------------------------------------------------------
 * Lesson "4.1: Behavior Planning"
 * Authors     : Benjamin Ulmer and Tobias Roth of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the inefficiency cost function used in the
 *                       trajectory planner.
 * ----------------------------------------------------------------------------
 */


#include "2_inefficiency_cost.h"


/* Computes the inefficiency cost of the goal speed given lane changes. 
 *
 * Here the inefficiency cost increases when trajectories with either 
 * `intended_lane` or `final_lane` have traffic moving slower than the
 * `target_speed`. The speed of each lane is given as a unitless number
 * indexed from the `lane_speeds` vector for each of the lane indices.
 * 
 * @param    target_speed   Desired speed (dimensionless) after manoeuvre. 
 * @param    intended_lane  Index of lane intended for this manoeuvre.
 * @param    final_lane     Index of lane immediately following manoeuvre.
 * @param    lane_speeds    Set of current speeds for each lane by index.
 * @returns  cost           Computed inefficiency cost for the trajectory.
 */
double inefficiency_cost(
    int target_speed,
    int intended_lane,
    int final_lane,
    const std::vector<int>& lane_speeds
) {
  // Compute the weighted change in speed from manoeuvres to target
  double delta_s = 2.0 * target_speed;
  delta_s = delta_s - lane_speeds[intended_lane] - lane_speeds[final_lane];
  // Return the ratio of weighted change in speed to target speed
  // i.e., `delta_s` is inversely proportional to the target speed.
  double cost = delta_s / target_speed;
  return cost;
}