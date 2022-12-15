#include "1_distance_cost.h"


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
) {
  // Compute the weighted lateral remaining distance to the goal lane
  double delta_d = 2.0 * goal_lane - intended_lane - final_lane;
  // Taking absolute value to get magnitude of total remaining distance
  double cost = 1.0 - exp(-(std::abs(delta_d) / distance_to_goal));
  return cost;
}