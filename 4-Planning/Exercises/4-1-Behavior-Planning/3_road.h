#ifndef 3_ROAD_H_
#define 3_ROAD_H_

#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <iostream>
#include <iterator>
#include "vehicle.h"

class Road {
 public:
  // Constructor
  Road(int speed_limit, double traffic_density, std::vector<int>& lane_speeds);

  // Destructor
  virtual ~Road();

  // Road functions
  Vehicle get_ego();
  void populate_traffic();
  void advance();
  void display(int timestep);
  void add_ego(int lane_num, int s, std::vector<int> &config_data);
  void cull();

  // Road variables
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