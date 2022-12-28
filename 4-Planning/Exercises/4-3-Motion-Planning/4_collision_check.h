/* ------------------------------------------------------------------------------
 * Lesson "4.3: Motion Planning"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the circle-based collision detector.
 * ----------------------------------------------------------------------------
 */

#include <math.h>
#include <array>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <vector>


/*** Global variables ***/
// Offsets of the three circles from the vehicle centre-point
// Each value is given in metres (m)
constexpr std::array<float, 3> CIRCLE_OFFSETS;
// Radiii of the three circles
// Each value is given in metres (m)
constexpr std::array<float, 3> CIRCLE_RADII;

/*** Struct definitions ***/
// Stores the 3D position on the road surface
// Each value is given in metres (m)
struct Location {
  float x;
  float y;
  float z;
};
// Stores the 3D rotation w.r.t. vehicle heading
// Each value is given in radians
struct Rotation {
  float pitch;
  float yaw;
  float roll;
};
// Stores the vehicle state instance
struct State {
  Location location;
  Rotation rotation;
};

/*** Global functions ***/
// Determines if a collision will occur between the current
// ego-vehicle state and all current obstacle states
std::vector<bool> collision_checker(
  const State& ego_vehicle,
  const std::vector<State>& obstacles
);