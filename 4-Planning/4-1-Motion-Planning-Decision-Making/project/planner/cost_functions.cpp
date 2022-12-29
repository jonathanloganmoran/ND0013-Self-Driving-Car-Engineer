/* ------------------------------------------------------------------------------
 * Project "4.1: Motion Planning and Decision Making for Autonomous Vehicles"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the cost functions used in the planner.
 * ----------------------------------------------------------------------------
 */

#include "cost_functions.h"


namespace cost_functions {
/* Goal-state trajectory difference cost function.
 *
 * Penalises trajectories whose coordinates (and derivatives) differ
 * from the goal-state trajectory.
 * 
 * @param    coeff
 * @param    duration     
 * @param    goals        Set of waypoints belonging to the goal-state.
 * @param    sigma        
 * @param    cost_weight  Value to weigh the cost by.
 * @returns  Weighted difference cost between given and goal-state trajectory.
 */
double diff_cost(
    std::vector<double> coeff, 
    double duration,
    std::array<double, 3> goals, 
    std::array<float, 3> sigma,
    double cost_weight
) {
  double cost = 0.0;
  std::vector<double> evals = utils::evaluate_f_and_N_derivatives(
      coeff, 
      duration, 
      2
  );
  // std::cout << "26 - Evaluating f and N derivatives Done. Size:" <<
  // evals.size() << "\n";
  for (size_t i = 0; i < evals.size(); i++) {
    double diff = fabs(evals[i] - goals[i]);
    cost += utils::logistic(diff / sigma[i]);
  }
  // std::cout << "diff_coeff Cost Calculated " << "\n";
  return cost_weight * cost;
}

/* Path-obstacle collision cost function.
 *
 * Penalises paths whose waypoints intersect with any object(s) by returning
 * an unweighted cost of `1.0`. If no collisions occur for the given `spiral`
 * path and set of `obstacles`, then the value `0.0` is returned.
 *
 * @param    spiral     Path containing the waypoints to evaluate.
 * @param    obstacles  Set of object states to evaluate.
 * @returns  Binary collision cost value.
 */
double collision_circles_cost_spiral(
    const std::vector<PathPoint>& spiral,
    const std::vector<State>& obstacles
) {
  bool collision{false};
  auto n_circles = CIRCLE_OFFSETS.size();
  for (auto wp : spiral) {
    // Iterate through all waypoints in the path
    if (collision) {
      // Collision occurred, skip checking remaining waypoints and return cost
      // LOG(INFO) << " ***** COLLISION DETECTED *********" << "\n";
      break;
    }
    // Get the current pose associated with this waypoint
    // NOTE: `theta` yaw angle is already given in radians (no need to convert)
    double cur_x = wp.x;
    double cur_y = wp.y;
    double cur_yaw = wp.theta;
    for (size_t c = 0; c < n_circles && !collision; ++c) {
      // Compute the centre-point for each circle representing the waypoint
      auto circle_center_x = cur_x + CIRCLE_OFFSETS[c] * std::sin(cur_yaw);
      auto circle_center_y = cur_y + CIRCLE_OFFSETS[c] * std::cos(cur_yaw);
      for (auto obst : obstacles) {
        if (collision) {
          break;
        }
        // Get the heading (yaw angle) of the obstacle
        auto actor_yaw = obst.rotation.yaw;
        for (size_t c2 = 0; c2 < n_circles && !collision; ++c2) {
          // Compute the centre-point for each circle representing the obstacle
          auto actor_center_x = (
              obst.location.x + CIRCLE_OFFSETS[c2] * std::cos(actor_yaw)
          );
          auto actor_center_y = (
              obst.location.y + CIRCLE_OFFSETS[c2] * std::sin(actor_yaw)
          );
          // Compute distance between obstacle- and ego-vehicle circle centres
          double dist = std::sqrt(
              std::pow((circle_center_x - actor_center_x), 2)
               + std::pow((circle_center_y - actor_center_y), 2)
          );
          // Evalaute if a collision will occur based on computed distance
          collision = (dist < (CIRCLE_RADII[c] + CIRCLE_RADII[c2]));
        }
      }
    }
  }
  return (collision) ? COLLISION : 0.0;
}


/* Distance to goal-state from final waypoint cost function.
 *
 * Paths whose final waypoint is situated closest to the goal-state,
 * i.e., the centre line of the lane, will be given a lower cost.
 * In other words, paths whose final waypoint is farther from the
 * centre line will be penalised.
 *
 * @param    spiral     Path containing the waypoints to evaluate.
 * @param    main_goal  Goal-state with position of the lane centre line.
 * @returns  cost       Distance to goal-state cost.
 */
double close_to_main_goal_cost_spiral(
    const std::vector<PathPoint>& spiral,
    State main_goal
) {
  auto n = spiral.size();
  // Calculate remaining distance between goal-state and last waypoint on path 
  auto delta_x = main_goal.location.x - spiral[n - 1].x;
  auto delta_y = main_goal.location.y - spiral[n - 1].y;
  auto delta_z = main_goal.location.z - spiral[n - 1].z;
  auto dist = std::sqrt(
      (delta_x * delta_x) 
      + (delta_y * delta_y) 
      + (delta_z * delta_z)
  );
  auto cost = utils::logistic(dist);
  // LOG(INFO) << "distance to main goal: " << dist;
  // LOG(INFO) << "cost (log): " << cost;
  return cost;
}
}  // namespace cost_functions