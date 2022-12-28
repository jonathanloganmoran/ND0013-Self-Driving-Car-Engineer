/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

#include "vehicle_dynamic_model.h"

#include <cfloat>
#include <cmath>

using namespace utils;

VehicleDynamicModel::VehicleDynamicModel() {}
VehicleDynamicModel::~VehicleDynamicModel() {}

void VehicleDynamicModel::load_vehicle_data(
    const SharedPtr<carla::client::Vehicle> vehicle) {
  // Get some basic Physical data from the vehicle
  auto physics_control = vehicle->GetPhysicsControl();
  GR = physics_control.final_ratio;
  r_e = physics_control.wheels[0].radius;
  m = physics_control.mass;
  c_a = physics_control.drag_coefficient;
  max_brake_torque = physics_control.wheels[0].max_brake_torque *
                     physics_control.wheels.size();
  max_steer_angle =
      utils::deg2rad(physics_control.wheels[0].max_steer_angle / 2.0);
}

double VehicleDynamicModel::get_throttle(
    const double accel_ref, const SharedPtr<carla::client::Vehicle> vehicle) {
  if (accel_ref < 0) return -0.5;

  auto ego_velocity_vec = vehicle->GetVelocity();
  auto v = magnitude(ego_velocity_vec);
  auto transform =
      boost::static_pointer_cast<carla::client::Actor>(vehicle)->GetTransform();
  auto pitch = deg2rad(transform.rotation.pitch);

  auto F_aero = c_a * v * v;
  auto R_x = c_r1 * v;
  auto F_g = m * g * CppAD::sin(pitch);
  auto F_load = F_aero + R_x + F_g;

  // ========= T Load =========
  auto T_load = GR * r_e * F_load;

  // ========= Throttle =========
  return T_load / (a_0 + a_1 * w_e + a_2 * w_e * w_e);
}

AD<double> VehicleDynamicModel::get_accel_engine(const AD<double>& throttle,
                                                 const AD<double>& velocity,
                                                 float pitch) {
  auto w_w = w_e * GR;
  AD<double> s;
  if (velocity * velocity < DBL_EPSILON)
    s = DBL_MAX;
  else
    s = (w_w * r_e - velocity) / velocity;

  AD<double> F_aero = c_a * velocity * velocity;
  AD<double> R_x = c_r1 * velocity;
  AD<double> F_g = m * g * CppAD::sin(pitch);
  AD<double> F_load = F_aero + R_x + F_g;

  AD<double> F_x;
  if (s * s < 1.0)
    F_x = c * s;
  else
    F_x = F_load;

  auto accel = (F_x - F_load) / m;
  return (accel < DBL_EPSILON ? throttle : accel);
}

AD<double> VehicleDynamicModel::get_decel_brake(const AD<double>& brake_ped,
                                                const AD<double>& velocity,
                                                float pitch) {
  AD<double> Treq = 0.01 * max_brake_torque * brake_ped;
  // auto Tb = 1/Kb * std::log(Kb*(Treq -
  AD<double> Fb = Treq / r_e;

  AD<double> F_aero = c_a * velocity * velocity;
  AD<double> R_x = c_r1 * velocity;
  AD<double> F_g = m * g * CppAD::sin(pitch);
  AD<double> F_load = F_aero + R_x + F_g;

  auto decel = (Fb + F_load) / m;
  return decel;
}

AD<double> VehicleDynamicModel::get_accel(
    const AD<double>& throttle, const AD<double>& velocity,
    const SharedPtr<carla::client::Vehicle> vehicle) {
  auto ego_transform =
      boost::static_pointer_cast<carla::client::Actor>(vehicle)->GetTransform();
  auto pitch = deg2rad(ego_transform.rotation.pitch);

  AD<double> accel{0.0};
  if (throttle < 0)  // we are braking
  {
    // getting the deceleration from the vehicle's brake model
    return get_decel_brake(throttle, velocity, pitch);
  } else {
    // getting the acceleration from the vehicle's engine model
    return get_accel_engine(throttle, velocity, pitch);
  }
}

double VehicleDynamicModel::get_throttle(const double velocity_ref,
                                         double pitch) {
  // Wheel Angular speed
  auto w_w = velocity_ref / r_e;

  // Engine Angular Speed
  auto w_e = w_w / GR;

  // Engine torque = T Load

  auto F_aero = c_a * v * v;
  auto R_x = c_r1 * v;
  auto F_g = m * g * CppAD::sin(pitch);
  auto F_load = F_aero + R_x + F_g;

  // ========= T Load =========
  auto T_load = GR * r_e * F_load;

  // Engine map lookup table Engine speed (RPM) Vs Engine Torque
  // I don't have the MAP
  return 0.01;
}