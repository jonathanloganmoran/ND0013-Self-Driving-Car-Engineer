/* ------------------------------------------------------------------------------
 * Lesson "4.1: Behavior Planning"
 * Authors     : Benjamin Ulmer and Tobias Roth of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the distance-goal cost function used
 *                       in the trajectory planner.
 * ----------------------------------------------------------------------------
 */


#ifndef 1_DISTANCE_COST_H_
#define 1_DISTANCE_COST_H_


#include <cmath>


/* Computes the distance cost to the goal state including lane changes.
 *
 * Here the cost increases with both the distance of intended lane from the
 * goal and the distance of the final lane form the goal. The cost also 
 * becomes larger as the vehicle gets closer to approaching the goal (i.e.,
 * the cost increases as longitudinal distance to the goal decreases).
 *
 * @param    goal_lane          Lane number containing the goal marker. 
 * @param    intended_lane      Lane number inteded for given manouevre.
 * @param    final_lane         Lane number immediately following manouevre. 
 * @param    distance_to_goal   Longitudinal distance from vehicle to goal.
 * @returns  cost               Associated distance cost of planned manoeuvre.
 */
double goal_distance_cost(
    int goal_lane, 
    int intended_lane, 
    int final_lane,
    double distance_to_goal
);


#endif  // 1_DISTANCE_COST_H
