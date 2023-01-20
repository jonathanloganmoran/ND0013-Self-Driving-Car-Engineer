/* ------------------------------------------------------------------------------
 * Lesson "4.4: Prediction"
 * Authors     : David Siller, Andrew Gray, Dominique Luna.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com) 
 *
 * Purpose of this file: Implements the Global Kinematic motion model.
 * ----------------------------------------------------------------------------
 */

#include "1_global_kinematic_model.h"

// Distance between the vehicle centre of mass and front axle
const double kLf = 2;


// Helper functions
double deg2rad(
    double x
) {
  return x * M_PI / 180.0; 
}
double rad2deg(
    double x
) {
  return x * 180.0 / M_PI; 
}


/* Returns the next-state determined by the global kinematic motion model.
 *
 * The `state` vector is assumed to have values in the following order:
 *   $[x_t, y_t, \psi_t, v_t]$,
 * where $x_t, y_t$ are the 2D position defined in the global reference frame,
 * and $\psi_t$ is the vehicle heading angle given in HERE.
 * The `actuators` vector is assumed to have values in the following order:
 *    $[\delta, a_t]$,
 * where $\delta$ is the steering angle input and $a_t$ is the acceleration
 * value normalised in range $[-1, 1] m/s^{2}$. 
 *
 * @param    state       State vector of values at time $t$.
 * @param    actuators   Control inputs to the system w.r.t. time $t$.
 * @param    dt          Delta-time, i.e., amount of elapsed time (s).
 * @returns  next_state  Updated state vector for time-step $t+1$. 
 */
Eigen::VectorXd global_kinematic(
    const Eigen::VectorXd& state, 
    const Eigen::VectorXd& actuators, 
    double dt
) {
  // Get the current state vector values
  double x_t = state(0);
  double y_t = state(1);
  double psi_t = state(2);
  double v_t = state(3);
  // Get the current actuator values (control inputs)
  double delta_t = actuators(0);
  double a_t = actuators(1);
  // Create a new vector for the next-state
  Eigen::VectorXd next_state(state.size());
  next_state(0) = x_t + v_t * std::cos(psi_t) * dt;
  next_state(1) = y_t + v_t * std::sin(psi_t) * dt;
  next_state(2) = psi_t + (v_t / kLf) * delta_t * dt;
  next_state(3) = v_t + a_t * dt;  
  return next_state;
}