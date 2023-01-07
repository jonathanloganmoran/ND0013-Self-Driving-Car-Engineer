/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/
/**
 * @file plot_utils.h
 **/

#pragma once

#include <vector>

#include <carla/client/Waypoint.h>
#include "matplotlibcpp.h"

#include "structs.h"

namespace utils {

void plotWaypoints(
    std::vector<carla::SharedPtr<carla::client::Waypoint>> waypoints);
void plotWaypointsAnimation(
    std::vector<carla::SharedPtr<carla::client::Waypoint>> waypoints);
template <typename T>
void plotAnimation(std::vector<T> xs, std::vector<T> ys);
void plot_goals(std::vector<State> goals);
void plot_spiral(std::vector<carla::geom::Transform> spiral);
}  // namespace utils