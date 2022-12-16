/* ------------------------------------------------------------------------------
 * Lesson "4.1: Behavior Planning"
 * Authors     : Benjamin Ulmer and Tobias Roth of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the `Vehicle` class and trajectory
 *                       planner. Computes the `trajectory`, i.e., a vector
 *                       containing the previous and updated `Vehicle` states
 *                       after a manoeuvre is made (e.g., "KL" — keep lane).
 * ----------------------------------------------------------------------------
 */


#ifndef 3_VEHICLE_H_
#define 3_VEHICLE_H_


#include "3_costs.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>


/* Implements the `Vehicle` class with respective state and kinematics variables.
 * 
 * @class   Vehicle   "3_vehicle.h"
 * @brief   Implements state and functions to handle trajectory updates.
 * @var     lane      Number of the lane the vehicle is in.
 * @var     s         Position the vehicle is at.
 * @var     v         Velocity the vehicle is travelling at.
 * @var     a         Acceleration applied to the vehicle.
 * @var     state     Next-state identifier, i.e., "CS" = constant speed.
 */
class Vehicle {
public:
  Vehicle() {};
  Vehicle(int lane, float s, float v, float a, std::string state="CS");
  virtual ~Vehicle();

  // Vehicle functions
  std::vector<Vehicle> choose_next_state(
      std::map<int, 
      std::vector<Vehicle>>& predictions
  );
  std::vector<std::string> successor_states();
  std::vector<Vehicle> generate_trajectory(
      std::string state,
      std::map<int, std::vector<Vehicle>>& predictions
  );
  std::vector<float> get_kinematics(
      std::map<int, std::vector<Vehicle>>& predictions, 
      int lane
  );
  std::vector<Vehicle> constant_speed_trajectory();
  std::vector<Vehicle> keep_lane_trajectory(
      std::map<int, std::vector<Vehicle>>& predictions
  );
  std::vector<Vehicle> lane_change_trajectory(
      std::string state,
      std::map<int, std::vector<Vehicle>>& predictions
  );
  std::vector<Vehicle> prep_lane_change_trajectory(
      std::string state,
      std::map<int, std::vector<Vehicle>>& predictions
  );
  void increment(
      int dt
  );
  float position_at(
      int t
  );
  bool get_vehicle_behind(
      std::map<int, std::vector<Vehicle>>& predictions, 
      int lane,
      Vehicle& rVehicle
  );
  bool get_vehicle_ahead(
      std::map<int, std::vector<Vehicle>>& predictions, 
      int lane,
      Vehicle& rVehicle
  );
  std::vector<Vehicle> generate_predictions(
      int horizon=2
  );
  void realize_next_state(
      std::vector<Vehicle>& trajectory
  );
  void configure(
      std::vector<int>& road_data
  );

  /* Manages the collision avoidance state.
   *
   * @struct  collider  "3_vehicle.h"
   * @brief   TODO.
   * @var     collision  Whether a collision has occurred. 
   * @var     time       At what time the collision occurred.
   */
  struct collider{
    bool collision;
    int  time;
  };

  // Initialise the vehicle lane directions
  // Here, moving left means increasing the current lane by one,
  // whereas moving right decreases the current lane by one
  std::map<std::string, int> lane_direction = {
      {"PLCL", 1}, 
      {"LCL", 1}, 
      {"LCR", -1}, 
      {"PLCR", -1}
  };

  int L = 1;
  // Buffer distance, affects `KL` (i.e., "keep lane" state) 
  int preferred_buffer = 6; // impacts "keep lane" behavior.
  int lane, s, goal_lane, goal_s, lanes_available;
  float v, target_speed, a, max_acceleration;
  std::string state;
};


#endif  // 3_VEHICLE_H_