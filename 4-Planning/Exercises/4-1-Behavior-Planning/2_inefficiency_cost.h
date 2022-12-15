#ifndef 2_INEFFICIENCY_COST_H_
#define 2_INEFFICIENCY_COST_H

#include <vector>


/* TODO.
 */
double inefficiency_cost(
    int target_speed,
    int intended_lane,
    int final_lane,
    const std::vector<int>& lane_speeds
);


#endif  // 2_INEFFICIENCY_COST_H
