#include "1_distance_cost.h"
#include "2_inefficiency_cost.h"
#include "3_road.h"
#include "3_vehicle.h"

#include <iostream>
#include <vector>


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
 * Where we have a `goal_lane = 0`, and three possible manoeuvres to
 * score in `intended_lane = 2` and `intended_lane = 1`. The distance
 * from `intended_lane = 2` to the `goal_lane = 0` can be e.g., 10m, 
 * while the distances from `intended_lane = 1` to the `goal_lane = 0`
 * can be 10m and e.g., 5m. 
 * 
 * Using our cost function, we want to give the two manoeuvres in lane 1 
 * a lower cost score since they are closer laterally to the goal lane.
 * However, within lane 1, we want to give the manoeuvre farther away from
 * the goal marker a lesser cost score (i.e., cost decreases as longitudinal
 * distance to goal increases). This is to factor in reaction time.
 * 
 * Therefore, the manoeuvre the highest cost will be for (2, 2, 10), i.e.,
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


/* Tests the inefficiency cost function.
 * Here the expectation is that the greater the change in speed
 * from the target speed will result in a higher inefficiency cost.
 * The higher the cost, the less desirable the trajectory. 
 */
void test_inefficiency_cost() {
  // Target speed of our vehicle
  int target_speed = 10;
  // Lane speeds for each lane
  std::vector<int> lane_speeds = {6, 7, 8, 9};
  // Test cases used for grading - do not change.
  double cost;
  std::cout << "Costs for (intended_lane, final_lane):" << "\n";
  std::cout << "---------------------------------------------------------" << "\n";
  cost = inefficiency_cost(target_speed, 3, 3, lane_speeds);
  std::cout << "The cost is " << cost << " for " << "(3, 3)" << "\n";
  cost = inefficiency_cost(target_speed, 2, 3, lane_speeds);
  std::cout << "The cost is " << cost << " for " << "(2, 3)" << "\n";
  cost = inefficiency_cost(target_speed, 2, 2, lane_speeds);
  std::cout << "The cost is " << cost << " for " << "(2, 2)" << "\n";
  cost = inefficiency_cost(target_speed, 1, 2, lane_speeds);
  std::cout << "The cost is " << cost << " for " << "(1, 2)" << "\n";
  cost = inefficiency_cost(target_speed, 1, 1, lane_speeds);
  std::cout << "The cost is " << cost << " for " << "(1, 1)" << "\n";
  cost = inefficiency_cost(target_speed, 0, 1, lane_speeds);
  std::cout << "The cost is " << cost << " for " << "(0, 1)" << "\n";
  cost = inefficiency_cost(target_speed, 0, 0, lane_speeds);
  std::cout << "The cost is " << cost << " for " << "(0, 0)" << "\n";
}


void test_behaviour_planner() {
  // impacts default behavior for most states
  int SPEED_LIMIT = 10;

  // all traffic in lane (besides ego) follow these speeds
  vector<int> LANE_SPEEDS = {6,7,8,9}; 

  // Number of available "cells" which should have traffic
  double TRAFFIC_DENSITY = 0.15;

  // At each timestep, ego can set acceleration to value between 
  //   -MAX_ACCEL and MAX_ACCEL
  int MAX_ACCEL = 2;

  // s value and lane number of goal.
  vector<int> GOAL = {300, 0};

  // These affect the visualization
  int FRAMES_PER_SECOND = 4;
  int AMOUNT_OF_ROAD_VISIBLE = 40;

  Road road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS);

  road.update_width = AMOUNT_OF_ROAD_VISIBLE;

  road.populate_traffic();

  int goal_s = GOAL[0];
  int goal_lane = GOAL[1];

  // configuration data: speed limit, num_lanes, goal_s, goal_lane, 
  //   and max_acceleration
  int num_lanes = LANE_SPEEDS.size();
  vector<int> ego_config = {SPEED_LIMIT,num_lanes,goal_s,goal_lane,MAX_ACCEL};
   
  road.add_ego(2,0, ego_config);
  int timestep = 0;
  
  while (road.get_ego().s <= GOAL[0]) {
    ++timestep;
    if (timestep > 100) {
      break;
    }
    road.advance();
    road.display(timestep);
    //time.sleep(float(1.0) / FRAMES_PER_SECOND);
  }

  Vehicle ego = road.get_ego();
  if (ego.lane == GOAL[1]) {
    cout << "You got to the goal in " << timestep << " seconds!" << endl;
    if(timestep > 35) {
      cout << "But it took too long to reach the goal. Go faster!" << endl;
    }
  } else {
    cout << "You missed the goal. You are in lane " << ego.lane 
         << " instead of " << GOAL[1] << "." << endl;
  }
}


int main() {
    // EX 4.1.1: Tests goal distance cost function on pre-defined test cases
    test_goal_distance_cost();
    // EX 4.1.2: Tests inefficiency cost function on pre-defined test cases
    test_inefficiency_cost();
    // EX 4.1.3: Tests the behaviour planner on pre-defined state variables
    test_behaviour_planner();
    return 0;
}