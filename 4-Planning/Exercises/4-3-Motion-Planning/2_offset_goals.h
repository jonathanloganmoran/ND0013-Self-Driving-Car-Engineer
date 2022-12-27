/* ------------------------------------------------------------------------------
 * Lesson "4.3: Motion Planning"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for goal-offset path planning functions.
 * ----------------------------------------------------------------------------
 */

#include <math.h>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>


/*** Global variables ***/
// Distance from the centre lane line
float _goal_offset;
// Number of offset paths to generate
int _num_goals;

/*** Struct definitions ***/
// Stores the 3D position on the road surface
// Each value is given in metres (m)
struct Location {
  float x;
  float y;
  float z;
};
// Stores the 3D rotation of the path w.r.t. vehicle heading
// Each value is given in radians
struct Rotation {
  float pitch;
  float yaw;
  float roll;
};
// Stores the path instance (i.e., deviation from the centre lane line)
struct State {
  Location location;
  Rotation rotation;
};

/*** Global functions ***/
// Generates the offset paths from the goal (centre lane line)
std::vector<State> generate_offset_goals(
    const State& goal_state
);