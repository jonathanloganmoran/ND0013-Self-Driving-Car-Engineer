/* ------------------------------------------------------------------------------
 * Project "5.1: Control and Trajectory Tracking for Autonomous Vehicles"
 * Authors     : Munir Jojo-Verge, Aaron Brown, Mathilda Badoual.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Executes the motion / trajectory planner from P4.1 
 *                       and the PID controller / trajectory tracker from P5.1.
 * ----------------------------------------------------------------------------
 */


#include "json.hpp"
#include "behavior_planner_FSM.h"
#include "motion_planner.h"
#include "planning_params.h"
#include "utils.h"
#include "pid_controller.h"
#include "Eigen/QR"
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <uWS/uWS.h>
#include <time.h>
#include <math.h>
#include <array>
#include <cfloat>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <fstream>
#include <typeinfo>
#include <limits>

using json = nlohmann::json;
#define _USE_MATH_DEFINES

/*** Setting the global variables **/
// Declares and initialises the ego-vehicle behaviour planner module
BehaviorPlannerFSM behavior_planner(
    P_LOOKAHEAD_TIME, 
    P_LOOKAHEAD_MIN, 
    P_LOOKAHEAD_MAX, 
    P_SPEED_LIMIT,
    P_STOP_THRESHOLD_SPEED, 
    P_REQ_STOPPED_TIME, 
    P_REACTION_TIME,
    P_MAX_ACCEL, 
    P_STOP_LINE_BUFFER
);
// Declares and initialises the ego-vehicle motion planner module
MotionPlanner motion_planner(
    P_NUM_PATHS, 
    P_GOAL_OFFSET, 
    P_ERR_TOLERANCE
);
// Sets the global obstacles flag to `false` (no obstacles at current position)
bool have_obst = false;
// Initialise the vector of obstacle states
std::vector<State> obstacles;


/* Helper function to check if the given string contains data.
 *
 * @param    s    String to examine for relevant data.
 * @returns  Substring with data of interest, empty string ("") otherwise. 
 */
std::string has_data(
    std::string s
) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("{");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (
      b1 != std::string::npos 
      && b2 != std::string::npos
  ) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


/* Creates the set of obstacles from the given coordinate vectors.
 *
 * A new `State` instance is created and initialised for every point in the
 * given coordinate vectors. Each new obstacle `State` is appended to the
 * `obstacles` vector and the global `obst_flag` is then set to `true`.
 *
 * @param  x_points   Coordinates of the obstacles along the x-axis.
 * @param  y_points   Coordinates of the obstacles along the y-axis.
 * @param  obstacles  Vector to update with obstacle `State` instances.
 * @param  obst_flag  Gloabl flag to update, is `true` when obstacles present.
 */
void set_obst(
    std::vector<double> x_points, 
    std::vector<double> y_points, 
    std::vector<State>& obstacles, 
    bool& obst_flag
) {
	for (int i = 0; i < x_points.size(); i++) {
		State obstacle;
		obstacle.location.x = x_points[i];
		obstacle.location.y = y_points[i];
		obstacles.push_back(obstacle);
	}
	obst_flag = true;
}


/* Returns the index of the point in vector closest to the given coordinates. 
 *
 * @param    point_x      Coordinate of the reference point along the x-axis. 
 * @param    point_y      Coordinate of the reference point along the y-axis.
 * @param    points_x     Set of x-coordinates to compute the distance to.
 * @param    points_y     Set of y-coordinates to compute the distance to.
 * @returns  closest_idx  Index of point in vector closest to the coordinates. 
 */
std::size_t get_closest_point_idx(
    double point_x,
    double point_y,
    std::vector<double> points_x,
    std::vector<double> points_y
) {
  // Index of closest found point 
  std::size_t closest_idx = 0;
  // Distance to point
  double dist_min = std::numeric_limits<double>::infinity();
  // Find closest point in vector
  for (std::size_t i = 0; i < points_x.size(); ++i) {
    double dist = std::pow(
        (std::pow((point_x - points_x[i]), 2)
         + std::pow((point_y - points_y[i]), 2)
        ),
        0.5
    );
    if (dist < dist_min) {
      dist_min = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}


/* Helper function to compute the angle between two points.
 *
 * @param  x1   Coordinate of the first 2D point along the x-axis.
 * @param  y1   Coordinate of the first 2D point along the y-axis.
 * @param  x2   Coordinate of the second 2D point along the x-axis.
 * @param  y2   Coordinate of the second 2D point along the y-axis.
 * @returns     Angle between the two points.
 */
double angle_between_points(
    double x1, 
    double y1, 
    double x2, 
    double y2
) {
  return atan2(y2 - y1, x2 - x1);
}


template <typename T> int sgn(
    T val
) {
  return (T(0) < val) - (val < T(0));
}


/* Updates the ego-vehicle trajectory.
 *
 * The trajectory is updated by interfacing with the behaviour and
 * motion planning modules in the `behavior_planner_FSM` and `motion_planner`
 * files, respectively. A state transition is computed in the behaviour module
 * for the given ego-vehicle state (position / orientation angle), which is
 * then used to compute the goal-offset trajectories as polynomial spirals in
 * the motion planner and velocity profile generator modules.
 * 
 * The best trajectory from the generated goal-offset trajectories is selected
 * and its corresponding waypoints are used to update the ego-vehicle.
 *
 * @param  x_points       Ego-vehicle trajectory coordinates along the x-axis.
 * @param  y_points       Ego-vehicle trajectory coordinates along the y-axis.
 * @param  v_points       Ego-vehicle trajectory velocity (m/s) values.
 * @param  yaw            Ego-vehicle trajectory yaw angle / heading.
 * @param  velocity       Current velocity (m/s) of the ego-vehicle.
 * @param  goal           Current goal-state of the ego-vehicle.
 * @param  is_junction    Junction state flag, `true` if at junction.
 * @param  tl_state       Traffic light state variable.
 * @param  spirals_x      Polynomial spiral coordinates along the x-axis.
 * @param  spirals_y      Polynomial spiral coordinates along the y-axis.
 * @param  spirals_v      Polynomial spiral velocity (m/s) values.
 * @param  best_spirals   Vector of indices of the best polynomial spirals.
 */
void path_planner(
    std::vector<double>& x_points, 
    std::vector<double>& y_points, 
    std::vector<double>& v_points, 
    double yaw, 
    double velocity, 
    State goal, 
    bool is_junction, 
    std::string tl_state, 
    std::vector<std::vector<double>>& spirals_x, 
    std::vector<std::vector<double>>& spirals_y, 
    std::vector<std::vector<double>>& spirals_v, 
    std::vector<int>& best_spirals
) {
  State ego_state;
  ego_state.location.x = x_points[x_points.size() - 1];
  ego_state.location.y = y_points[y_points.size() - 1];
  ego_state.velocity.x = velocity;
  if (x_points.size() > 1) {
  	ego_state.rotation.yaw = angle_between_points(
        x_points[x_points.size() - 2], 
        y_points[y_points.size() - 2], 
        x_points[x_points.size() - 1], 
        y_points[y_points.size() - 1]
    );
  	ego_state.velocity.x = v_points[v_points.size() - 1];
  	if (velocity < 0.01) {
  		ego_state.rotation.yaw = yaw;
    }
  }
  Maneuver behavior = behavior_planner.get_active_maneuver();
  goal = behavior_planner.state_transition(
      ego_state, 
      goal, 
      is_junction, 
      tl_state
  );
  if (behavior == STOPPED) {
  	int max_points = 20;
  	double point_x = x_points[x_points.size() - 1];
  	double point_y = y_points[x_points.size() - 1];
  	while (x_points.size() < max_points) {
  	  x_points.push_back(point_x);
  	  y_points.push_back(point_y);
  	  v_points.push_back(0);
  	}
  	return;
  }
  auto goal_set = motion_planner.generate_offset_goals(
      goal
  );
  auto spirals = motion_planner.generate_spirals(
      ego_state, 
      goal_set
  );
  auto desired_speed = utils::magnitude(
      goal.velocity
  );
  State lead_car_state;  // = to the vehicle ahead...
  if (spirals.size() == 0) {
  	std::cout << "Error: No spirals generated " << "\n";
  	return;
  }
  for (int i = 0; i < spirals.size(); i++) {
    auto trajectory = (
        motion_planner._velocity_profile_generator.generate_trajectory(
            spirals[i], 
            desired_speed, 
            ego_state,
            lead_car_state, 
            behavior
        )
    );
    std::vector<double> spiral_x;
    std::vector<double> spiral_y;
    std::vector<double> spiral_v;
    for (int j = 0; j < trajectory.size(); j++) {
      double point_x = trajectory[j].path_point.x;
      double point_y = trajectory[j].path_point.y;
      double velocity = trajectory[j].v;
      spiral_x.push_back(point_x);
      spiral_y.push_back(point_y);
      spiral_v.push_back(velocity);
    }
    spirals_x.push_back(spiral_x);
    spirals_y.push_back(spiral_y);
    spirals_v.push_back(spiral_v);
  }
  best_spirals = motion_planner.get_best_spiral_idx(
      spirals, 
      obstacles, 
      goal
  );
  int best_spiral_idx = -1;
  if (best_spirals.size() > 0) {
  	best_spiral_idx = best_spirals[best_spirals.size() - 1];
  }
  int index = 0;
  int max_points = 20;
  int add_points = spirals_x[best_spiral_idx].size();
  while (
      x_points.size() < max_points 
      && index < add_points
  ) {
    double point_x = spirals_x[best_spiral_idx][index];
    double point_y = spirals_y[best_spiral_idx][index];
    double velocity = spirals_v[best_spiral_idx][index];
    index++;
    x_points.push_back(point_x);
    y_points.push_back(point_y);
    v_points.push_back(velocity);
  }
}


/* Runs the motion / trajectory planning (P4.1) and the
 * control / trajectory tracking (P5.1) modules.
 * 
 * The ego-vehicle state is updated relative to the response of the
 * PID controller, which is implemented for both steering and throttle
 * commands. The error between the current- and intended trajectory is
 * computed on each time-step and used to update the PID controller.
 * 
 * The output steering / throttle comamnds are saved to the respective
 * text files, which are later used to plot the performance of the controller
 * using the `plot_pid.py` script.
 *
 * Lastly, the actuation commands are issued to the ego-vehicle via the
 * `uWebSockets` library. These commands and any obstacles are rendered in
 * the CARLA Simulator environment in a separate process.
 */
int main() {
  std::cout << "Starting server" << "\n";
  uWS::Hub h;
  double new_delta_time;
  int i = 0;
  // Read in steering data for the PID controller
  fstream file_steer;
  file_steer.open(
      "steer_pid_data.txt", 
      std::ofstream::out | std::ofstream::trunc
  );
  file_steer.close();
  // Read in throttle data for the PID controller
  fstream file_throttle;
  file_throttle.open(
      "throttle_pid_data.txt", 
      std::ofstream::out | std::ofstream::trunc
  );
  file_throttle.close();
  // Create a new timer instance
  time_t prev_timer;
  time_t timer;
  time(&prev_timer);
  /*** Initialise the PID controller for the ego-vehicle throttle commands ***/
  PID pid_throttle = PID();
  // CANDO: Set appropriate gain values here
  // CASE 1 : Using the P-controller (proportional-gain only):
  // pid_throttle.init_controller(1.0, 0.0, 0.0, 1.0, -1.0);
  // pid_throttle.init_controller(0.5, 0.0, 0.0, 1.0, -1.0);
  // CASE 2 : Using the PD-controller (proportional-derivative gain only):
  // pid_throttle.init_controller(1.0, 0.0, 1.0, 1.0, -1.0);
  // pid_throttle.init_controller(0.5, 0.001, 0.0, 1.0, -1.0);
  // CASE 3 : Using the PID-controller (proportional-integral-derivative gain):
  // pid_throttle.init_controller(1.0, 1.0, 1.0, 1.0, -1.0);
  // pid_throttle.init_controller(0.5, 0.001, 0.1, 1.0, -1.0);
  // pid_throttle.init_controller(0.21, 0.0009, 0.1, 1.0, -1.0);
  // Final run (I achieved the best results, i.e., no collisions, with these)
  pid_throttle.init_controller(0.21, 0.0006, 0.080, 1.0, -1.0);
  /*** Initialise the PID controller for the ego-vehicle steering commands ***/
  PID pid_steer = PID();
  // CANDO: Set appropriate gain values here
  // CASE 1 : Using the P-controller (proportional-gain only):
  // pid_steer.init_controller(1.0, 0.0, 0.0, 1.2, -1.2);
  // pid_steer.init_controller(0.5, 0.0, 0.0, 1.2, -1.2);
  // CASE 2 : Using the PD-controller (proportional-derivative gain only):
  // pid_steer.init_controller(1.0, 0.0, 1.0, 1.2, -1.2);
  // pid_steer.init_controller(0.5, 0.001, 0.0, 1.2, -1.2);
  // CASE 3 : Using the PID-controller (proportional-integral-derivative gain):
  // pid_steer.init_controller(1.0, 1.0, 1.0, 1.2, -1.2);
  // pid_steer.init_controller(0.5, 0.001, 0.1, 1.2, -1.2);
  // pid_steer.init_controller(0.29, 0.0011, 0.3, 1.2, -1.2);
  // Final run (I achieved the best results, i.e., no collisions, with these)
  // NOTE: the min / max steering angle has been reduced to $\pm 0.6 rad$ in
  // order to clip steering commands to feasible values (approx. 35 degrees)
  pid_steer.init_controller(0.3, 0.0025, 0.17, 0.60, -0.60);
  h.onMessage(
      [
        &pid_steer, 
        &pid_throttle, 
        &new_delta_time, 
        &timer, 
        &prev_timer, 
        &i, 
        &prev_timer
      ](
          uWS::WebSocket<uWS::SERVER> ws, 
          char *data, 
          size_t length, 
          uWS::OpCode opCode
) {
    auto s = has_data(data);
    if (s != "") {
      auto data = json::parse(s);
      /*** Saving steering / throttle commands to output files ****/
      // Create file to save steering values from PID controller
      fstream file_steer;
      file_steer.open("steer_pid_data.txt");
      // Create file to save throttle values from PID controller
      fstream file_throttle;
      file_throttle.open("throttle_pid_data.txt");
      /*** Fetching the trajectory and current waypoint data ***/
      std::vector<double> x_points = data["traj_x"];
      std::vector<double> y_points = data["traj_y"];
      std::vector<double> v_points = data["traj_v"];
      double waypoint_x = data["waypoint_x"];
      double waypoint_y = data["waypoint_y"];
      double waypoint_t = data["waypoint_t"];
      bool is_junction = data["waypoint_j"];
      /*** Fetching the ego-vehicle state data ***/
      double x_position = data["location_x"];
      double y_position = data["location_y"];
      double z_position = data["location_z"];
      double yaw = data["yaw"];
      double velocity = data["velocity"];
      /*** Fetching the environment state data ***/
      double sim_time = data["time"];
      std::string tl_state = data["tl_state"];
      if (!have_obst) {
        std::vector<double> x_obst = data["obst_x"];
        std::vector<double> y_obst = data["obst_y"];
        set_obst(x_obst, y_obst, obstacles, have_obst);
      }
      State goal;
      goal.location.x = waypoint_x;
      goal.location.y = waypoint_y;
      goal.rotation.yaw = waypoint_t;
      std::vector<std::vector<double>> spirals_x;
      std::vector<std::vector<double>> spirals_y;
      std::vector<std::vector<double>> spirals_v;
      std::vector<int> best_spirals;
      path_planner(
          x_points, 
          y_points, 
          v_points, 
          yaw, 
          velocity, 
          goal, 
          is_junction, 
          tl_state, 
          spirals_x, 
          spirals_y, 
          spirals_v, 
          best_spirals
      );
      // Save current time and compute the delta-time value
      time(&timer);
      new_delta_time = difftime(timer, prev_timer);
      prev_timer = timer;
      ////////////////////////////////////////
      // Steering control
      ////////////////////////////////////////
      // Update the delta-time variable w.r.t. the previous steering command
      pid_steer.update_delta_time(new_delta_time);
      // Get the index of the closest point in the reference trajectory to the
      // current vehicle position w.r.t. the PID controller steering command
      std::size_t idx_closest_point = get_closest_point_idx(
          x_position,
          y_position,
          x_points,
          y_points
      );
      // Compute the steering angle error
      // i.e., difference in heading between current- and trajectory position 
      double error_steer = angle_between_points(
          x_position,
          y_position,
          x_points[idx_closest_point],
          y_points[idx_closest_point]
      ) - yaw;
      // Compute the steering control value to apply
      pid_steer.update_error(error_steer);
      double steer_output = pid_steer.total_error();
      // Save the output steering command data to the text file
      file_steer.seekg(std::ios::beg);
      for (int j=0; j < i - 1; ++j) {
        file_steer.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      }
      file_steer << i << " " << error_steer;
      file_steer << " " << steer_output << "\n";
      ////////////////////////////////////////
      // Throttle control
      ////////////////////////////////////////
      // Update the delta-time variable w.r.t. the previous throttle command
      pid_throttle.update_delta_time(new_delta_time);
      // Compute throttle error
      // i.e., the difference between current- and trajectory velocity
      double error_throttle = v_points[idx_closest_point] - velocity;
      // Compute the throttle and brake control values to apply
      double throttle_output;
      double brake_output;
      pid_throttle.update_error(error_throttle);
      // Get the response of the PID controller
      double throttle_response = pid_throttle.total_error();
      if (throttle_response > 0.0) {
        // Set positive response values as actual throttle output command 
        throttle_output = throttle_response;
        brake_output = 0;
      } 
      else {
        // Set negative response values to brake output command
        throttle_output = 0;
        brake_output = -throttle_response;
      }
      // Save the output throttle command data to the text file
      file_throttle.seekg(std::ios::beg);
      for(int j=0; j < i - 1; ++j) {
          file_throttle.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
      }
      file_throttle << i << " " << error_throttle;
      file_throttle << " " << brake_output;
      file_throttle << " " << throttle_output << "\n";
      // Execute the control commands with a new message
      json msgJson;
      msgJson["brake"] = brake_output;
      msgJson["throttle"] = throttle_output;
      msgJson["steer"] = steer_output;
      msgJson["trajectory_x"] = x_points;
      msgJson["trajectory_y"] = y_points;
      msgJson["trajectory_v"] = v_points;
      msgJson["spirals_x"] = spirals_x;
      msgJson["spirals_y"] = spirals_y;
      msgJson["spirals_v"] = spirals_v;
      msgJson["spiral_idx"] = best_spirals;
      msgJson["active_maneuver"] = behavior_planner.get_active_maneuver();
      // Set the minimum point threshold before performing update
      // NOTE: For a high update rate use value `19`,
      //       For a slow update rate use value `4`
      msgJson["update_point_thresh"] = 16;
      auto msg = msgJson.dump();
      i += 1;
      file_steer.close();
      file_throttle.close();
      ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    }
  });
  h.onConnection(
      [](
          uWS::WebSocket<uWS::SERVER> ws, 
          uWS::HttpRequest req
  ) {
    std::cout << "Connected!!!" << "\n";
  });
  h.onDisconnection(
      [&h](
          uWS::WebSocket<uWS::SERVER> ws, 
          int code, 
          char *message, 
          size_t length
  ) {
    ws.close();
    std::cout << "Disconnected" << "\n";
  });
  int port = 4567;
  if (h.listen("0.0.0.0", port)) {
    std::cout << "Listening to port " << port << "\n";
    h.run();
  }
  else {
    std::cerr << "Failed to listen to port" << "\n";
    return -1;
  }
}
