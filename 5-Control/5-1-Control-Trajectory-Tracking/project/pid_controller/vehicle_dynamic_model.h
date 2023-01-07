/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

#pragma once

#include <carla/client/Vehicle.h>
#include <cppad/cppad.hpp>

#include "utils.h"

template <typename T>
using SharedPtr = boost::shared_ptr<T>;

using CppAD::AD;

class VehicleDynamicModel {
 private:
  /* data */
 public:
  VehicleDynamicModel();
  ~VehicleDynamicModel();
  double get_throttle(const double accel_ref,
                      const SharedPtr<carla::client::Vehicle> vehicle);
  double get_throttle(const double velocity_ref, double pitch);

  AD<double> get_accel(const AD<double>& throttle, const AD<double>& velocity,
                       const SharedPtr<carla::client::Vehicle> vehicle);
  void load_vehicle_data(const SharedPtr<carla::client::Vehicle> vehicle);

  AD<double> get_decel_brake(const AD<double>& brake_ped,
                             const AD<double>& velocity, float pitch);
  AD<double> get_accel_engine(const AD<double>& throttle,
                              const AD<double>& velocity, float pitch);
  ///============
  // Default Parameters
  // ============

  // Throttle to engine torque
  double a_0{400};
  double a_1{0.1};
  double a_2{-0.0002};

  // Throttle details
  double max_throttle_angle{utils::deg2rad(90)};

  // Gear ratio, effective tire radius, mass + inertia
  double GR{0.35};
  double r_e{0.3};
  double J_e{10};
  double m{2000};
  double g{9.81};

  // Aerodynamic and friction coefficients
  double c_a{1.36};
  double c_r1{0.01};

  // Tire force
  double c{10000};
  double F_max{10000};

  // State variables
  double x{0};
  double v{5};
  double a{0};
  double w_e{100};
  double w_e_dot{0};

  // Brake system
  double max_brake_torque{0.0};

  double max_steer_angle{0.0};
};