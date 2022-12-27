/* ------------------------------------------------------------------------------
 * Lesson "4.3: Motion Planning"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the goal-offset path planning functions.
 *                       The new path for the vehicle is computed as a
 *                       deviation of the orignal path w.r.t. centre lane-line.
 * ----------------------------------------------------------------------------
 */

#include "2_offset_goals.h"


/*** Global variables ***/
// Distance from the centre lane line
float _goal_offset = 1.0;
// Number of offset paths to generate
int _num_goals = 7;

/*** Struct definitions ***/
// Stores the 3D position on the road surface
// Each value is given in metres (m)
struct Location {
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
};
// Stores the 3D rotation of the path w.r.t. vehicle heading
// Each value is given in radians
struct Rotation {
  float pitch = 0.0;
  float yaw = 0.0;
  float roll = 0.0;
};
// Stores the path instance (i.e., deviation from the centre lane line)
struct State {
  Location location;
  Rotation rotation;
};


/* Generates the offset paths from the goal (centre lane line).
 *
 * The generated paths are aligned on a perpendicular line to the heading of
 * the original goal state. The paths are offset from the centre lane line at
 * a distance `_goal_offset`. The offset $x$- and $y$-coordinates are computed
 * using a simple trigonometric relation, i.e.,
 * $x_{offset} = x_{centre_goal} + goal_nr * offset_dist * cos(\phi + \pi / 2)$,
 * $y_{offset} = y_{centre_goal} + goal_nr * offset_dist * cos(\phi + \pi / 2)$.
 * 
 * Here the `goal_nr` is defined as an element in the `_num_goals` vector,
 * which are the positions on the left and right of the centre, i.e.,
 * where `goal_nr = 0`. The `_num_goals` values to the left of the centre line
 * are negative, wheras the values to the right of the centre line are positive.
 *
 * @param    goal_state     Starting 3D position and rotation of centre lane.
 * @returns  goals_offset   3D positions / rotations deviated from centre.
 */
std::vector<State> generate_offset_goals(
    const State& goal_state
) {
  // Create a vector of `_num_goals` path offsets,
  // each are a deviation from the centre lane line (i.e., the goal state) 
  std::vector<State> goals_offset;
  // Get the yaw angle perpendicular to the goal state heading (centre line)
  // from which to compute the path direction offsets from
  // Note: pi / 2 = `M_PI_2`
  float yaw_plus_90 = goal_state.rotation.yaw + M_PI_2;
  // Compute the path offsets
  for (int goal_nr = 0; goal_nr < _num_goals; ++goal_nr) {
    auto goal_offset = goal_state;
    // 1. Calculate the offset offset distance for goal "i"
    float offset_product = (goal_nr - (int)(_num_goals / 2)) * _goal_offset;
    // 2. Calculate the goal position along the $x$- and $y$-axes
    // The offset path is defined perpendicular to the goal state (centre line),
    // so we calculate the offset coordinates w.r.t. this perpendicular yaw direction
    goal_offset.location.x += offset_product * std::cos(yaw_plus_90);
    goal_offset.location.y += offset_product * std::sin(yaw_plus_90);
    // UNCOMMENT TO PRINT THE COORDINATE AND YAW ANGLE OFFSET VALUES
    // std::cout << "x: " << goal_offset.location.x << "\n";
    // std::cout << "y: " << goal_offset.location.y << "\n";
    // std::cout << "yaw: " << goal_offset.rotation.yaw << "\n";
    goals_offset.push_back(goal_offset);
  }
  // Return the set of waypoints offset from the goal
  return goals_offset;
}