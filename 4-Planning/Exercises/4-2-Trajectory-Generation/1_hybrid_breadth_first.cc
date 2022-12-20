/* ------------------------------------------------------------------------------
 * Lesson "4.2: Trajectory Generation"
 * Authors     : Sebastian Thrun, Emmanuel Boidot of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the Hybrid A* search algorithm.
 *                       Here a breadth-first search over the discretised map
 *                       space is performed which is guided by a continuous
 *                       vehicle control state. This guarantees kinematic
 *                       feasibility of the generated trajectory.
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


/* Expands the given state in the search space. 
 *
 * @param    state        Starting state to expand from.
 * @returns  next_states  Vector of expanded next-states.
 */
std::vector<HBF::maze_s> HBF::expand(
    HBF::maze_s& state
) {
  // Get the starting state variables
  int g = state.g;
  double x = state.x;
  double y = state.y;
  double theta = state.theta;
  // Incremeent the g-cost for the heuristic evaluation
  int g2 = g + 1;
  std::vector<HBF::maze_s> next_states;

  for (int i = -35; i < 40; i += 5) {
    double delta_i = double(i);
    double delta = M_PI / 180.0 * delta_i;
    double omega = SPEED / LENGTH * tan(delta);
    double theta2 = theta + omega;
    if(theta2 < 0) {
      theta2 += 2*M_PI;
    }
    double x2 = x + SPEED * cos(theta);
    double y2 = y + SPEED * sin(theta);
    HBF::maze_s state2;
    state2.g = g2;
    state2.x = x2;
    state2.y = y2;
    state2.theta = theta2;
    next_states.push_back(state2);
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
 * NOTE: currently the BFS does not include a heuristic function
 * evaluation and is therefore inefficient. 
 * 
 * TODO is a modification of the BFS to perform heuristic evaluation
 * as in Hybrid A* search.
 *
 * @param    grid   Discretised grid space in 2D.
 * @param    start  State (2D position and heading) to begin search from.
 * @param    goal   Coordinates of the goal location in 2D grid space.
 * @returns  path   Path from the `start` to `goal` discovered with BFS. 
 */
HBF::maze_path HBF::search(
    std::vector<std::vector<int>>& grid,
    std::vector<double>& start, 
    std::vector<int>& goal
) {
  // TODO: Add heuristics and convert this function into hybrid A*
  std::vector<std::vector<std::vector<int>>> closed(
    NUM_THETA_CELLS, 
    std::vector<std::vector<int>>(grid[0].size(), 
    std::vector<int>(grid.size()))
  );
  std::vector<std::vector<std::vector<maze_s>>> came_from(
    NUM_THETA_CELLS, 
    std::vector<std::vector<maze_s>>(grid[0].size(), 
    std::vector<maze_s>(grid.size()))
  );
  // Create a new starting `State`
  double theta = start[2];
  int stack = theta_to_stack_number(theta);
  int g = 0;
  maze_s state;
  state.g = g;
  state.x = start[0];
  state.y = start[1];
  state.theta = theta;
  // Close the visited `State` tuple
  closed[stack][idx(state.x)][idx(state.y)] = 1;
  // Add the visited state to the trajectory history
  came_from[stack][idx(state.x)][idx(state.y)] = state;
  int total_closed = 1;
  // Add the new state to the list of states to explore
  std::vector<maze_s> opened = {state};
  bool finished = false;
  // Explore the remaining states in `opened`
  while(!opened.empty()) {
    // TODO: Sort by ascending f-value as given by the heuristic function
    // Then, get the state with the lowest f-value
    // For now, we grab the first element
    maze_s current = opened[0];
    opened.erase(opened.begin());
    int x = current.x;
    int y = current.y;
    // Check if the goal has been reached
    if (
      (idx(x) == goal[0]) 
      && (idx(y) == goal[1])
    ) {
      std::cout << "Found path to goal in " << total_closed;
      std::cout << " expansions" << "\n";
      maze_path path;
      path.came_from = came_from;
      path.closed = closed;
      path.final = current;
      return path;
    }
    // Get the set of next-possible states
    std::vector<maze_s> next_state = expand(current);

    for (int i = 0; i < next_state.size(); ++i) {
      // Explore each next-state 
      int g2 = next_state[i].g;
      double x2 = next_state[i].x;
      double y2 = next_state[i].y;
      double theta2 = next_state[i].theta;
      if (
        (x2 < 0 || x2 >= grid.size())
        || ((y2 < 0 || y2 >= grid[0].size())
      ) {
        // Skip this state if not within grid dimensions
        continue;
      }
      // Check that this state has not previously been visited
      // and that no obstacle exists at the state location
      int stack2 = theta_to_stack_number(theta2);
      if(
        (closed[stack2][idx(x2)][idx(y2)] == 0) 
        && (grid[idx(x2)][idx(y2)] == 0)
       ) {
        // Add the valid state to the explored states stack
        opened.push_back(next_state[i]);
        closed[stack2][idx(x2)][idx(y2)] = 1;
        came_from[stack2][idx(x2)][idx(y2)] = current;
        ++total_closed;
      }
    }
  }
  std::cout << "No valid path." << "\n";
  HBF::maze_path path;
  path.came_from = came_from;
  path.closed = closed;
  path.final = state;
  return path;
}