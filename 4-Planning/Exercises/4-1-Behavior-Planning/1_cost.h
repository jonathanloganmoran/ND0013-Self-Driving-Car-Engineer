#ifndef COST_H
#define COST_H


#include <cmath>


double goal_distance_cost(
    int goal_lane, 
    int intended_lane, 
    int final_lane,
    double distance_to_goal
);


#endif  // COST_H
