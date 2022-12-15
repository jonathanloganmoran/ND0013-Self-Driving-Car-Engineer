#include "1_distance_cost.h"

#include <iostream>


/* Tests the goal-distance cost function.
 * The costs are given for input (`intended_lane`, `final_lane`, `goal_lane`).
 * 
 * For example, we might have the following lane configuration:
 *
 * --------------------------------------------------------------
 *  3 |
 * --------|-----------------------------------------------------
 *  2 |    o
 * --------|------------------|----------------------------------
 *  1 |    o                  o
 * -----------------------------------------------------|--------
 *  0 |                                                 x
 * --------------------------------------------------------------
 * 
 * Where we have a `goal_lane = 0`, and three possible manouevres to
 * score in `intended_lane = 2` and `intended_lane = 1`. The distance
 * from `intended_lane = 2` to the `goal_lane = 0` can be e.g., 10m, 
 * while the distances from `intended_lane = 1` to the `goal_lane = 0`
 * can be 10m and e.g., 5m. 
 * 
 * Using our cost function, we want to give the two manouevres in lane 1 
 * a lower cost score since they are closer laterally to the goal lane.
 * However, within lane 1, we want to give the manouevre farther away from
 * the goal marker a lesser cost score (i.e., cost decreases as longitudinal
 * distance to goal increases). This is to factor in reaction time.
 * 
 * Therefore, the manouevre the highest cost will be for (2, 2, 10), i.e.,
 * position in lane 2 with no intended or final lane change closer to goal.
 * Assuming an intention to move into lane 1, i.e., (2, 1, 10), this will
 * be our second highest cost. The next highest will be for (1, 1, 5),
 * which represents position in lane 1 with shorter distance to goal and no
 * intention of moving into goal lane. After that, we have (1, 0, 5), which
 * is the position in lane 1 with intention of moving into goal lane but with
 * shorter distance. This results in a lowest cost score assigned to
 * (1, 0, 10), which represents the position in lane 1 with larger distance to
 * goal and intention of moving into goal lane. This is the most optimal case.
 */
void test_goal_distance_cost() {
  // Integer number of the goal lane (see docstring for example)
  int goal_lane = 0;
  double cost;
  std::cout << "Costs for (intended_lane, final_lane, goal_distance):" << "\n";
  std::cout << "---------------------------------------------------------" << "\n";
  cost = goal_distance_cost(goal_lane, 2, 2, 1.0);
  std::cout << "The cost is " << cost << " for " << "(2, 2, 1.0)" << "\n";
  cost = goal_distance_cost(goal_lane, 2, 2, 10.0);
  std::cout << "The cost is " << cost << " for " << "(2, 2, 10.0)" << "\n";
  cost = goal_distance_cost(goal_lane, 2, 2, 100.0);
  std::cout << "The cost is " << cost << " for " << "(2, 2, 100.0)" << "\n";
  cost = goal_distance_cost(goal_lane, 1, 2, 100.0);
  std::cout << "The cost is " << cost << " for " << "(1, 2, 100.0)" << "\n";
  cost = goal_distance_cost(goal_lane, 1, 1, 100.0);
  std::cout << "The cost is " << cost << " for " << "(1, 1, 100.0)" << "\n";
  cost = goal_distance_cost(goal_lane, 0, 1, 100.0);
  std::cout << "The cost is " << cost << " for " << "(0, 1, 100.0)" << "\n";
  cost = goal_distance_cost(goal_lane, 0, 0, 100.0);
  std::cout << "The cost is " << cost << " for " << "(0, 0, 100.0)" << "\n";
    
}


int main() {
    // EX 4.1.1: Tests goal distance cost function on pre-defined test cases
    test_goal_distance_cost();
    return 1;
}