/* ------------------------------------------------------------------------------
 * Lesson "4.3: Motion Planning"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the circle-based collision detector.
 * ----------------------------------------------------------------------------
 */

#include "4_collision_check.h"

/*** Global variables ***/
// Offsets of the three circles from the vehicle centre-point
// Each value is given in metres (m)
constexpr std::array<float, 3> CIRCLE_OFFSETS = {-1.0, 1.0, 2.0};
// Radiii of the three circles
// Each value is given in metres (m)
constexpr std::array<float, 3> CIRCLE_RADII = {1.5, 1.5, 1.5};

/*** Struct definitions ***/
// Stores the 3D position on the road surface
// Each value is given in metres (m)
struct Location {
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
};
// Stores the 3D rotation w.r.t. vehicle heading
// Each value is given in radians
struct Rotation {
  float pitch = 0.0;
  float yaw = 0.0;
  float roll = 0.0;
};
// Stores the vehicle state instance
struct State {
  Location location;
  Rotation rotation;
};


/* Checks if a collision occurs between the ego-vehicle and obstacles.
 *
 * The ego-vehicle and obstacles / actors are assumed to be represented
 * by a conservative circle-based collision detection method, i.e., the
 * entire area of the ego-vehicle and all obstacles / actors are enclosed
 * fully by a set of three circles associated to each object. Each of the three
 * circles is placed with an offset from the centre-point of the object given
 * in metres by `CIRCLE_OFFSETS`. Each of the circles has a radius given in
 * metres by `CIRCLE_RADII`.
 * 
 * In order to determine if a collision occurs, the distance between the
 * ego-vehicle's circles and all obstacle / actor circles are calculated w.r.t.
 * the centre-point of each circle. If the distance between these ego- and actor
 * circles is less than the sum of their respective radii, then a collision is
 * said to occur. In this case, the function returns the boolean `true`.
 *
 * @param    ego_vehicle    Current position / orientation of the ego-vehicle. 
 * @param    obstacles      Vector of obstacle states (position / orientation).
 * @returns  collision      Boolean indicating whether a collision occurrs.
 */
std::vector<bool> collision_checker(
    const State& ego_vehicle,
    const std::vector<State>& obstacles
) {
  std::vector<bool> collisions(obstacles.size(), false);
  auto n_circles = CIRCLE_OFFSETS.size();
  bool collision{false};
  for (size_t obs = 0; obs < obstacles.size(); ++obs) {
    auto actor = obstacles[obs];
    for (size_t c = 0; c < n_circles && !collision; ++c) {
      // Compute the circle centre-offset points for the ego-vehicle
      Location ego_circle_center;
      ego_circle_center.x = (
        ego_vehicle.location.x
        + CIRCLE_OFFSETS[c] * std::cos(ego_vehicle.rotation.yaw)
      );
      ego_circle_center.y = (
        ego_vehicle.location.y
        + CIRCLE_OFFSETS[c] * std::sin(ego_vehicle.rotation.yaw)
      );
      for (size_t c2 = 0; c2 < n_circles && !collision; ++c2) {
        // Compute the circle centre-offset points for the actor (obstacle)
        Location actor_circle_center;
        actor_circle_center.x = (
          actor.location.x
          + CIRCLE_OFFSETS[c2] * std::cos(actor.rotation.yaw)
        );
        actor_circle_center.y = (
          actor.location.y
          + CIRCLE_OFFSETS[c2] * std::sin(actor.rotation.yaw)
        );
        // Compute the distance between the ego-vehicle and obstacle / actor
        double dist = std::sqrt(
          std::pow((ego_circle_center.x - actor_circle_center.x), 2) 
          + std::pow((ego_circle_center.y - actor_circle_center.y), 2)
        );
        // Check for collision between ego-vehicle and obstacle / actor
        // i.e., collision occurs if distance between ego- and actor- centres
        // is less than the sum of their circle radii. 
        collision = dist < (CIRCLE_RADII[c] + CIRCLE_RADII[c2]);  
        // UNCOMMENT TO PRINT TRUTH VALUE FOR THIS CIRCLE AND ITS COORDINATE VALUES
        // std::cout << "c " << c << ", c2 " << c2 << " distance: " << dist;
        // std::cout << " c_x: " << ego_circle_center.x;
        // std::cout << " c_y: " << ego_circle_center.y;
        // std::cout << " a_x: " << actor_circle_center.x;
        // std::cout << " a_y: " << actor_circle_center.y;
        // std::cout << ", RadiiSum: " << (CIRCLE_RADII[c] + CIRCLE_RADII[c2]);
        // std::cout << ", collision: " << collision << "\n";
      }
    }
    collisions[obs] = collision;
    // UNCOMMENT TO PRINT TRUTH VALUE FOR THIS OBSTACLE
    // std::cout << "obs " << obs << ": " << collision << "\n";
  }
  return collisions;
}