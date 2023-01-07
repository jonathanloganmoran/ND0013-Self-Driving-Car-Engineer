/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file structs.h
 **/

#pragma once

#include <vector>

#include "carla/geom/Location.h"
#include "carla/geom/Rotation.h"

using namespace std;

namespace cg = carla::geom;

enum Maneuver  // A.K.A State
{
  // Follow Lane - Maintain a constant speed = SpeedLimit-buffer in the same
  // lane
  FOLLOW_LANE,

  // Following - Disntance with Vehicle in front is < d_min
  // and we need to slow down/speed Up (calc accel) behind it until we can
  // Change lane or move at the desired speed
  FOLLOW_VEHICLE,

  DECEL_TO_STOP,

  STOPPED
};

struct State {
  cg::Location location;
  cg::Rotation rotation;
  cg::Vector3D velocity;
  cg::Vector3D acceleration;
};

struct MPC_State {
  float x;
  float y;
  float yaw;
  float v;
  MPC_State(float x_, float y_, float yaw_, float v_) {
    x = x_;
    y = y_;
    yaw = yaw_;
    v = v_;
  };
};

struct maneuver_params {
  int dir;              // directiion: +1(left), 0 (stay in lane), -1 (right)
  double target_x;      // target final s coordinate. Ego_s + maneuver len
  double target_speed;  // target end_speed of the maneuver
  double duration;      // target duration
};

struct PathPoint {
  // coordinates
  double x;
  double y;
  double z;

  // direction on the x-y plane
  double theta;
  // curvature on the x-y planning
  double kappa;
  // accumulated distance from beginning of the path
  double s;

  // derivative of kappa w.r.t s.
  double dkappa;
  // derivative of derivative of kappa w.r.t s.
  double ddkappa;
};

struct TrajectoryPoint {
  // path point
  PathPoint path_point;

  // linear velocity
  double v;  // in [m/s]
  // linear acceleration
  double a;
  // relative time from beginning of the trajectory
  double relative_time;
};

struct SpiralConfig {
  int simpson_size = 9;
  double newton_raphson_tol = 0.01;
  int newton_raphson_max_iter = 20;
};
