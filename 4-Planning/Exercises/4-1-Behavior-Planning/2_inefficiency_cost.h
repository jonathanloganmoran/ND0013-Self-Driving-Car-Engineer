#ifndef 2_INEFFICIENCY_COST_H_
#define 2_INEFFICIENCY_COST_H

#include <vector>


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
);


#endif  // 2_INEFFICIENCY_COST_H
