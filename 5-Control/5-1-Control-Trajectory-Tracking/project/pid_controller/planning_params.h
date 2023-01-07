/* ------------------------------------------------------------------------------
 * Project "5.1: Control and Trajectory Tracking for Autonomous Vehicles"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Defines the parameters of the hierarchial planner.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <array>


/*** Planning Constants ***/
// Number of paths (goals) to create
// CANDO: Modify the number of path offsets to generate
#define P_NUM_PATHS 5                  // Number of deviations to enumerate
// Minimum lookhead distance (m)
#define P_LOOKAHEAD_MIN 8.0
// Maximum lookahead distance (m)
#define P_LOOKAHEAD_MAX 20.0
// Lookahead time (s)
#define P_LOOKAHEAD_TIME 1.5
// Goal-offset distance (m)
#define P_GOAL_OFFSET 1.0
// Error tolerance (m)
#define P_ERR_TOLERANCE 0.1
// Time-gap (s) to enforce
#define P_TIME_GAP 1.0
// Maximum permitted accelertion (m/s^2)
#define P_MAX_ACCEL 1.5
// Speed (m/s) to use for slow manoeuvre
#define P_SLOW_SPEED 1.0
// Speed limit (m/s) to set in `FOLLOW_LANE` state
#define P_SPEED_LIMIT 3.0
// Distance (m) to buffer from stopping point (stay behind)
#define P_STOP_LINE_BUFFER 0.5
// Maximum speed (m/s) to set for stopping manoeuvre
#define P_STOP_THRESHOLD_SPEED 0.02
// Minimum required stopping time (s)
#define P_REQ_STOPPED_TIME 1.0
// Lookahead distance (m) to use for lead vehicle
#define P_LEAD_VEHICLE_LOOKAHEAD 20.0
// Minimum required reaction time (s)
#define P_REACTION_TIME 0.25
// Number of waypoints to use in polynomial spiral (path)
// CANDO: Modify the number of waypoints to use in each path
// NOTE: Needs to be sufficiently large to avoid compile error
#define P_NUM_POINTS_IN_SPIRAL 25
// Minimum distance (m) needed for full stop
// NOTE: Replaces motion controller (velocity-based) for now
#define P_STOP_THRESHOLD_DISTANCE \
  P_LOOKAHEAD_MIN / P_NUM_POINTS_IN_SPIRAL * 2

/*** Circle-Based Collision Detection Parameters ***/
// Longitudinal offsets of the three circles from the vehicle centre-point
// Each value is given in metres (m)
constexpr std::array<float, 3> CIRCLE_OFFSETS = {-1.0, 1.0, 3.0};
// Radii of the three circles
// Each value is given in metres (m)
constexpr std::array<float, 3> CIRCLE_RADII = {1.5, 1.5, 1.5};

/*** Goal-state Pertubation Parameters ***/
// Time deviation (s)
constexpr double dt = 0.05;
// Standard deviation along x-axis in ego-vehicle reference frame
// Used to generate perturbations of `x`, `x_dot`, `x_double_dot`
constexpr std::array<float, 3> SIGMA_X = {4, 1.0, 2.0};
// Stabdard deviation along y-axis in ego-vehicle reference frame
// Used to generate perturbations of `y`, `y_dot`, `y_double_dot`
constexpr std::array<float, 3> SIGMA_Y = {0.5, 1.0, 0.5};
// Standard deviation of yaw in ego-vehicle reference frame
// Used to generate perturbations of `yaw_{x}`, `yaw_{y}`, `yaw_{z}`
constexpr std::array<float, 3> SIGMA_YAW = {0.17, 1.0, 1.0};
// Standard deviation of elapsed time (s)
// i.e., deviation in amount of time required to finish manoeuvre
constexpr double SIGMA_T = 0.5;

/*** Jerk / Acceleration Parameters ****/
// Filtered jerk (m/s^3) over one second w.r.t. comfort constraints
constexpr double CONFORT_MAX_LAT_JERK = 0.9;
constexpr double CONFORT_MAX_LON_JERK = 1.5;
constexpr double CONFORT_ACCUM_LON_JERK_IN_ONE_SEC = 3.0;
constexpr double CONFORT_ACCUM_LAT_JERK_IN_ONE_SEC = 2.0;
// Acceleration (m/s^2) accumulated over one second w.r.t. comfort constraints
constexpr double CONFORT_ACCUM_LON_ACC_IN_ONE_SEC = 1.0;
constexpr double CONFORT_ACCUM_LAT_ACC_IN_ONE_SEC = 0.6;
// Maximum permitted acceleration (m/s^2)
constexpr double CONFORT_MAX_LON_ACCEL = 3.0;
constexpr double CONFORT_MAX_LAT_ACCEL = 1.0;
// Permitted minimum and maximum manoeuvre time (time-steps) 
constexpr double MIN_MANEUVER_TIME = dt * 10;
constexpr double MAX_MANEUVER_TIME = dt * 75;
