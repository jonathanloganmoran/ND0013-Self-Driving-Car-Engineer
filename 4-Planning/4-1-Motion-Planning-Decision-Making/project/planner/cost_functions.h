/* ------------------------------------------------------------------------------
 * Project "4.1: Motion Planning and Decision Making for Autonomous Vehicles"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the functions used in the planner.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include "planning_params.h"
#include "structs.h"
#include "utils.h"
#include <carla/client/Client.h>
#include <glog/logging.h>
#include <math.h>
#include <algorithm>                // std::max, min (not yet used)
#include <cfloat>
#include <iostream>
#include <limits>                   // std::numeric_limits (not yet used)
#include <numeric>                  // std::accumulate (not yet used)
#include <vector>

using namespace std;
using namespace utils;
namespace cc = carla::client;
template <typename T>
using SharedPtr = boost::shared_ptr<T>;

// Priority levels for costs:
// Higher `COST` will give you a higher priority to `AVOID`
const double TIME_DIFF = 1e1;
const double X_DIFF = 1e12;
const double Y_DIFF = 1e12;
const double EFFICIENCY = 1e3;
const double MAX_JERK = 1e8;
const double TOTAL_JERK = 1e7;
const double COLLISION = std::numeric_limits<double>::infinity();
const double DANGER = 1e3;
const double MAX_ACCEL = 1e8;
const double TOTAL_ACCEL = 1e8;
const double RIGHT_LANE_CHANGE = 1e1;
// Average vehicle {Length, Width} in metres (m)
// (https://en.wikipedia.org/wiki/Family_car)
const vector<double> VEHICLE_SIZE = {5, 2};
// Minimum distance needed in case of congestion (i.e., when $v = 0$)
const double MIN_FOLLOW_DISTANCE = 1 * VEHICLE_SIZE[0];


namespace cost_functions {
// Goal-state trajectory difference cost
double diff_cost(
    std::vector<double> coeff, 
    double duration,
    std::array<double, 3> goals, 
    std::array<float, 3> sigma,
    double cost_weight
);
// Binary path-obstacle collision cost
double collision_circles_cost_spiral(
    const std::vector<PathPoint>& spiral,
    const std::vector<State>& obstacles
);
// Distance to goal-state from final waypoint cost
double close_to_main_goal_cost_spiral(
    const std::vector<PathPoint>& spiral,
    State goal
);
}  // namespace cost_functions