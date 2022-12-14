/* ------------------------------------------------------------------------------
 * Lesson "4.2: Trajectory Generation"
 * Authors     : Sebastian Thrun, Emmanuel Boidot of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the Hybrid A* search (Dolgov et al., 2008).
 *                       Here a breadth-first search over the discretised map
 *                       space is performed. The search is guided by a cost / 
 *                       heuristic function minimising the Manhattan distance
 *                       to the goal position. The vehicle trajectories through
 *                       the maze are given by a continuous control state which
 *                       is modelled by the kinematic bicycle motion model for
 *                       a discretised vehicle heading in range [0, 2*pi]. 
 *                       This guarantees kinematic feasibility of the generated
 *                       trajectory.
 * ----------------------------------------------------------------------------
 */

#include "1_hybrid_breadth_first.h"


// Initialise HBF constructor and destructor
HBF::HBF() {}
HBF::~HBF() {}


/* Returns the corresponding 'stack' in 3D configuration space of the angle.
 *
 * Here the vehicle heading angle `theta` in the interval [0, +/- 2*pi] is
 * discretised, i.e., partitioned into `NUM_THETA_CELLS`, such that the lower
 * discretised `theta` values are assigned lower stack numbers, whereas the
 * higher discretised `theta` values closer to 2*pi are assigned to larger
 * stack numbers.
 * 
 * @param    theta          Heading angle of the vehicle.
 * @returns  stack_number   Corresponding angle in 3D configuration space.
 */
int HBF::theta_to_stack_number(
    double theta
) {
  double new_theta = fmod(
    (theta + 2 * M_PI),
    (2 * M_PI)
  );
  int stack_number = (int)(
    round(new_theta * NUM_THETA_CELLS / (2*M_PI))
    ) % NUM_THETA_CELLS;
  return stack_number;
}


/* Returns the corresponding grid index of the continuous position. 
 *
 * @param    float_num  Continuous position to obtain grid index of. 
 * @returns  Corresponding index in the discretised grid.
 */
int HBF::idx(
    double float_num
) {
  return int(floor(float_num));
}


/* Computes the Manhattan distance to the goal state.
 *
 * @param    x      Current position along the x-axis.
 * @param    y      Current position along the y-axis.
 * @param    goal   Goal state with desired position.
 * @returns  goal
 */
double HBF::heuristic(
    double x, 
    double y,
    std::vector<int>& goal 
) {
  return fabs(x - goal[0]) + fabs(y - goal[1]);
}


/* Expands the given state in the search space. 
 *
 * @param    state        Starting state to expand from.
 * @returns  next_states  Vector of expanded next-states.
 */
std::vector<HBF::maze_s> HBF::expand(
    HBF::maze_s& state,
    std::vector<int>& goal
) {
  double theta = state.theta;
  std::vector<HBF::maze_s> next_states;
  // Create trajectories using the kinematic bicycle motion model
  for (int i = -35; i < 40; i += 5) {
    double delta_i = double(i);
    // Convert degrees to radians
    double delta_rad = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta_rad);
    double next_theta = theta + omega;
    // Normalise theta in range [0, 2*pi]
    if (next_theta < 0) {
      next_theta += 2 * M_PI;
    }
    double next_x = state.x + SPEED * cos(theta);
    double next_y = state.y + SPEED * sin(theta);
    HBF::maze_s next_state;
    next_state.g = state.g + 1;
    next_state.f = next_state.g + heuristic(next_x, next_y, goal);
    next_state.x = next_x;
    next_state.y = next_y;
    next_state.theta = next_theta;
    next_states.push_back(next_state);
  }
  return next_states;
}


/* Returns the path of nodes to the `final` from `start`. 
 *
 * @param    came_from  3D map from the previous vehicle states.
 * @param    start      Starting state vector to obtain path from.
 * @param    final      Final state vector to obtain path to.
 * @returns  path       Path of previous states. 
 */
std::vector< HBF::maze_s> HBF::reconstruct_path(
  std::vector<std::vector<std::vector<HBF::maze_s>>>& came_from, 
  std::vector<double>& start, 
  HBF::maze_s& final
) {
  std::vector<maze_s> path = {final};
  int stack = theta_to_stack_number(final.theta);
  maze_s current = came_from[stack][idx(final.x)][idx(final.y)];
  stack = theta_to_stack_number(current.theta);  
  double x = current.x;
  double y = current.y;
  while (
    (x != start[0])
    || (y != start[1])
  ) {
    path.push_back(current);
    current = came_from[stack][idx(x)][idx(y)];
    x = current.x;
    y = current.y;
    stack = theta_to_stack_number(current.theta);
  }
  return path;
}


/* Implements the breadth-first search algorithm.
 *
 * Here the BFS includes a heuristic function evaluation which minimises
 * the Manhattan distance from a current state to the goal position.
 * 
 * A 2D input `grid` map is given s.t. each position on the grid corresponds
 * to an integer value "0" for free-space, or "1" for obstacle.
 *
 * @param    grid   Binary-valued (1: obtacle, 0: free space) grid space in 2D.
 * @param    start  State (2D position and heading) to begin search from.
 * @param    goal   Coordinates of the goal location in 2D grid space.
 * @returns  path   Path from the `start` to `goal` discovered with BFS. 
 */
HBF::maze_path HBF::search(
    std::vector<std::vector<int>>& grid,
    std::vector<double>& start, 
    std::vector<int>& goal
) {
  // Add heuristics and convert this function into hybrid A*
  // Initialise the 3D maps from the discretised heading angle and 2D `grid`
  std::vector<std::vector<std::vector<int>>> states_closed(
    NUM_THETA_CELLS, 
    std::vector<std::vector<int>>(grid[0].size(), 
    std::vector<int>(grid.size()))
  );
  std::vector<std::vector<std::vector<maze_s>>> states_came_from(
    NUM_THETA_CELLS, 
    std::vector<std::vector<maze_s>>(grid[0].size(), 
    std::vector<maze_s>(grid.size()))
  );
  // Create a new starting `maze_s` state
  double theta = start[2];
  int stack = theta_to_stack_number(theta);
  int g = 0;
  maze_s state;
  state.g = g;
  state.f = g + heuristic(start[0], start[1], goal);
  state.x = start[0];
  state.y = start[1];
  state.theta = theta;
  // Close the visited state tuple
  states_closed[stack][idx(state.x)][idx(state.y)] = 1;
  // Add the visited state to the trajectory history
  states_came_from[stack][idx(state.x)][idx(state.y)] = state;
  int total_closed = 1;
  // Add the new state to the list of states to explore
  std::vector<maze_s> states_opened = {state};
  bool finished = false;
  // Explore the remaining states in `opened`
  while(!states_opened.empty()) {
    // Sort by ascending f-value as given by the heuristic function
    sort(states_opened.begin(), states_opened.end(), 
      [&](maze_s s1, maze_s s2) {
        return s1.f < s2.f;
    });
    // Then, get the state with the lowest f-value
    // TODO: Replace vector with `std::deque` (optimised for pop from front)
    maze_s current = states_opened[0];
    states_opened.erase(states_opened.begin());
    // Check if the goal has been reached
    if (
      (idx(current.x) == goal[0]) 
      && (idx(current.y) == goal[1])
    ) {
      std::cout << "Found path to goal in " << total_closed;
      std::cout << " expansions" << "\n";
      maze_path path;
      path.states_came_from = states_came_from;
      path.states_closed = states_closed;
      path.final = current;
      return path;
    }
    // Get the set of next-possible states
    std::vector<maze_s> next_state = expand(current, goal);
    for (int i = 0; i < next_state.size(); ++i) {
      // Explore each next-state 
      int next_g = next_state[i].g;
      double next_x = next_state[i].x;
      double next_y = next_state[i].y;
      double next_theta = next_state[i].theta;
      if (
        (next_x < 0 || next_x >= grid.size())
        || (next_y < 0 || next_y >= grid[0].size())
      ) {
        // Skip this state if not within grid dimensions
        continue;
      }
      // Check that this state has not previously been visited
      // and that no obstacle exists at the state location
      int stack_num = theta_to_stack_number(next_theta);
      if(
        (states_closed[stack_num][idx(next_x)][idx(next_y)] == 0) 
        && (grid[idx(next_x)][idx(next_y)] == 0)
       ) {
        // Add the valid state to the explored states stack
        states_opened.push_back(next_state[i]);
        states_closed[stack_num][idx(next_x)][idx(next_y)] = 1;
        states_came_from[stack_num][idx(next_x)][idx(next_y)] = current;
        ++total_closed;
      }
    }
  }
  std::cout << "No valid path." << "\n";
  HBF::maze_path path;
  path.states_came_from = states_came_from;
  path.states_closed = states_closed;
  path.final = state;
  return path;
}