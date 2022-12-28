/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file cost_functions.h
 **/

#pragma once

#include <math.h>

#include <algorithm>  // std::max, min
#include <cfloat>
#include <iostream>
#include <limits>
#include <numeric>  // std::accumulate
#include <vector>

#include <carla/client/Client.h>
#include <glog/logging.h>

#include "planning_params.h"
#include "structs.h"
#include "utils.h"

using namespace std;
using namespace utils;
namespace cc = carla::client;

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

// priority levels for costs: Higher COST will give you a higher priority to
// AVOID

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

// Average Vehicle {Length, Width} in meters
// (https://en.wikipedia.org/wiki/Family_car)
const vector<double> VEHICLE_SIZE = {5, 2};

// is the minimum distance in case of congestion (v = 0).
const double MIN_FOLLOW_DISTANCE = 1 * VEHICLE_SIZE[0];

namespace cost_functions {
// COST FUNCTIONS

double diff_cost(vector<double> coeff, double duration,
                 std::array<double, 3> goals, std::array<float, 3> sigma,
                 double cost_weight);

double collision_circles_cost_spiral(const std::vector<PathPoint>& spiral,
                                     const std::vector<State>& obstacles);

double close_to_main_goal_cost_spiral(const std::vector<PathPoint>& spiral,
                                      State goal);

}  // namespace cost_functions