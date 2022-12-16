/* ------------------------------------------------------------------------------
 * Lesson "4.1: Behavior Planning"
 * Authors     : Benjamin Ulmer and Tobias Roth of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the `Road` class which manages a simulated 
 *                       highway environment and its state variables.
 *                       The functions defined here are used by the trajectory
 *                       planner to discover possible manoeuvres that minimise
 *                       a set of objectives given the respective state of the
 *                       ego- or shadowed vehicle.
 * ----------------------------------------------------------------------------
 */


/* Initialises a new highway environment.
 * 
 * @param  speed_limit      Fixed speed limit (ego-vehicle will not exceed).
 * @param  traffic_density  Amount of traffic on road (not used yet).
 * @param  lane_speeds      Current speed of vehicles in this lane.
 * @var    num_lanes        Total number of lanes on highway.
 * @var    camera_center    Half-width of camera view, positions the vehicle.
 */
Road::Road(
    int speed_limit, 
    double traffic_density, 
    std::vector<int>& lane_speeds
) {
  this->num_lanes = lane_speeds.size();
  this->lane_speeds = lane_speeds;
  this->speed_limit = speed_limit;
  this->density = traffic_density;
  this->camera_center = this->update_width / 2;
}

// The destructor
Road::~Road() {}


/* Fetches the ego-vehicle state object from vector of current vehicles.
 * 
 * @returns a `Vehicle` state object belonging to the ego-vehicle.
 */
Vehicle Road::get_ego() {
  return this->vehicles.find(this->ego_key)->second;
}


/* Initialises the highway lanes with vehicles and sets their states.
 */
void Road::populate_traffic() {
  int start_s = std::max(
    this->camera_center - (this->update_width / 2), 
    0
  );
  for (int lane = 0; lane < this->num_lanes; ++lane) {
    int lane_speed = this->lane_speeds[lane];
    bool vehicle_just_added = false;
    for (int s = start_s; s < start_s + this->update_width; ++s) {
      if (vehicle_just_added) {
        vehicle_just_added = false;
      }
      if (((double)rand() / (RAND_MAX)) < this->density) {
        // If the randomly-chosen "density" is less than max density 
        Vehicle vehicle = Vehicle(lane, 
                                  s, 
                                  lane_speed, 
                                  0
        );
        // Set the vehicle state to keep 'constant speed'
        vehicle.state = "CS";
        this->vehicles_added += 1;
        this->vehicles.insert(
            std::pair<int, Vehicle>(vehicles_added, vehicle)
        );
        vehicle_just_added = true;
      }
    }
  }
}


/* Advances the environment to the next time-step.
 *
 * Here the trajectory predictions and current state are updated.
 * Then, the next-best trajectory is selected and realised (i.e,
 * the trajectory is roughly simulated to the next-state step).
 */
void Road::advance() {
  std::map<int, std::vector<Vehicle>> predictions;
  std::map<int, Vehicle>::iterator it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    int v_id = it->first;
    std::vector<Vehicle> preds = it->second.generate_predictions();
    predictions[v_id] = preds;
    ++it;
  }
  it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    int v_id = it->first;
    if (v_id == ego_key) {
      // Update the ego-vehicle trajectories for the given predictions
      std::vector<Vehicle> trajectory = it->second.choose_next_state(
          predictions
      );
      it->second.realize_next_state(trajectory);
    }
    else {
      it->second.increment(1);
    }
    ++it;
  } 
}


/* Updates the ego-vehicle state to the given lane and position.
 * 
 * @param  lane_num     Number of lane to place ego-vehicle in.
 * @param  s            Position to place vehicle relative to goal.
 * @param  config_data  Values for target speed, num. lanes, goal info, etc.
 */
void Road::add_ego(
    int lane_num, 
    int s, 
    std::vector<int>& config_data
) {
  std::map<int, Vehicle>::iterator it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    int v_id = it->first;
    Vehicle v = it->second;
    if (v.lane == lane_num && v.s == s) {
      // Remove the ego-vehicle at the previous state
      this->vehicles.erase(v_id);
    }
    ++it;
  }
  // Create the new ego-vehicle state object
  Vehicle ego = Vehicle(lane_num, 
                        s, 
                        this->lane_speeds[lane_num], 
                        0
  );
  // Configure the new state with the given parameters
  ego.configure(config_data);
  // Set the ego-vehicle state to 'keep lane'
  ego.state = "KL";
  // Insert the go-vehicle onto the "map"
  this->vehicles.insert(
      std::pair<int, Vehicle>(ego_key, ego)
  );
}


/* Prints the highway environment for the given time-step number.
 * 
 * Here the lanes in the highway environment are represented as
 * ASCII-valued strings. The ego-vehicle is represented with a unique
 * ASCII-valued marker, and its distance away from the goal marker
 * is printed.
 *
 * @param  timestep  Current time-step number to print.
 */
void Road::display(
    int timestep
) {
  Vehicle ego = this->vehicles.find(this->ego_key)->second;
  int s = ego.s;
  std::string state = ego.state;
  this->camera_center = std::max(
      s, 
      this->update_width / 2
  );
  int s_min = std::max(
      this->camera_center - this->update_width / 2, 
      0
  );
  int s_max = s_min + this->update_width;
  std::vector<std::vector<std::string>> road;
  for (int i = 0; i < this->update_width; ++i) {
    std::vector<std::string> road_lane;
    for (int ln = 0; ln < this->num_lanes; ++ln) {
      road_lane.push_back("     ");
    }
    road.push_back(road_lane);
  }
  std::map<int, Vehicle>::iterator it = this->vehicles.begin();
  while (it != this->vehicles.end()) {
    int v_id = it->first;
    Vehicle v = it->second;
    if (s_min <= v.s && v.s < s_max) {
      std::string marker = "";
      if (v_id == this->ego_key) {
        marker = this->ego_rep;
      } 
      else {
        std::stringstream oss;
        std::stringstream buffer;
        buffer << " ";
        oss << v_id;
        for (int buffer_i = oss.str().length(); buffer_i < 3; ++buffer_i) {
          buffer << "0";
        }
        buffer << oss.str() << " ";
        marker = buffer.str();
      }
      road[int(v.s - s_min)][int(v.lane)] = marker;
    }
    ++it;
  }
  std::ostringstream oss;
  oss << "+Meters ======================+ step: " << timestep << "\n";
  int i = s_min;
  for (int lj = 0; lj < road.size(); ++lj) {
    if (i % 20 ==0) {
      std::stringstream buffer;
      std::stringstream dis;
      dis << i;
      for (int buffer_i = dis.str().length(); buffer_i < 3; ++buffer_i) {
        buffer << "0";
      }
      oss << buffer.str() << dis.str() << " - ";
    } 
    else {
      oss << "      ";
    }          
    ++i;
    for (int li = 0; li < road[0].size(); ++li) {
      oss << "|" << road[lj][li];
    }
    oss << "|";
    oss << "\n";
  }
  std::cout << oss.str();
}