/* ------------------------------------------------------------------------------
 * Lesson "4.1: Behavior Planning"
 * Authors     : Benjamin Ulmer and Tobias Roth of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the `Road` class managing a simulated 
 *                       highway environment and its state variables.
 * ----------------------------------------------------------------------------
 */


#ifndef 3_ROAD_H_
#define 3_ROAD_H_

#include "3_vehicle.h"
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <sstream>
#include <vector>


/* Defines the highway environment and state-related variables and functions.
 *
 * The lanes on the road are assigned a numerical integer id, beginning with
 * the right-most lane from the ego-vehicle assigned to value `0`. The lane id
 * increases with each lane left of the right-most lane. Note that the flow of
 * traffic in all lanes have the same direction of travel, East, with respect
 * to the ego-vehicle heading. All position, velocity and acceleration values
 * are given in 1D.
 *
 * @class  Road    "3_road.h"
 * @brief  Defines the highway environment and state variables / functions. 
 * @var    speed_limit      Maximum speed ego-vehicle must not exceed.
 * @var    traffic_density  Ratio of vehicles on road, in range [0, 1].
 * @var    lane_speeds      Vector of current moving speeds for each lane. 
 * @var    num_lanes        Total number of lanes on highway.
 * @var    update_width     Width of the camera view.
 * @var    camera_center    Half-width of camera view.
 * @var    ego_key          Identifier for the ego-vehicle.
 * @var    vehicles_added   Number of vehicles initialised on the highway.
 * @var    ego_rep          Ego-vehicle marker in string format for printing.
 * @var    vehicles         Vector of `Vehicle` state objects.
 */
class Road {
 public:
  Road(int speed_limit, 
       double traffic_density, 
       std::vector<int>& lane_speeds
  );
  virtual ~Road();

  Vehicle get_ego();
  void populate_traffic();
  void advance();
  void display(int timestep);
  void add_ego(int lane_num, int s, std::vector<int> &config_data);
  void cull();

  int update_width = 70;
  int vehicles_added = 0;
  int ego_key = -1;
  int num_lanes, speed_limit, camera_center;
  double density; 
  std::map<int, Vehicle> vehicles;
  std::string ego_rep = " *** ";
  std::vector<int> lane_speeds; 
};


#endif  // 3_ROAD_H_