/* ------------------------------------------------------------------------------
 * Project "5.1: Control and Trajectory Tracking for Autonomous Vehicles"
 * Authors     : Mathilde Badoual.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the PID controller.
 * ----------------------------------------------------------------------------
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <vector>
#include <iostream>
#include <math.h>


/* The PID controller class.
 *
 * Implements the proportional-integral-derivative (PID) controller for use in
 * vehicle trajectory tracking. The response of the PID controller is used to
 * execute actuations via steering and throttle commands which track a
 * reference trajectory. The PID controller is expressed mathematically as:
 * $$\begin{align}
 * \alpha &= -\tau_{p} * \mathrm{CTE} 
 *           - \tau_{d} * \Delta \mathrm{CTE} 
 *           - \tau_{i} * \int_{0}^{t} \mathrm{CTE},
 * \end{align}$$
 * where the integral term $\int_{0}^{t} \mathrm{CTE}$ is given as the sum of
 * the instantaneous error over time. This gives the accumulated offset that
 * should have been previously corrected.
 * 
 * @var  lim_max_output  Maximum output value (used to threshold controller).
 * @var  lim_min_output  Minimum output value (used to threshold controller). 
 * @var  delta_t         Elapsed time (s) (used to compute derivative term).
 * @var  error_p         Proportional error term.
 * @var  error_i         Integral error term.
 * @var  error_d         Derivative error term.
 * @var  k_p             Proportional gain value (used to compute `error_p`)
 * @var  k_i             Integral gain value (used to compute `error_i`)
 * @var  k_d             Derivative gain value (used to compute `error_d`)
 */
class PID {
 public:
  double lim_max_output;
  double lim_min_output;
  double delta_t;
  double error_p;
  double error_i;
  double error_d;
  double k_p;
  double k_i;
  double k_d;

  PID();
  virtual ~PID();

  // Initialises the PID controller with the given parameter values
  void PID::init_controller(
      double k_p, 
      double k_i, 
      double k_d, 
      double lim_max_output, 
      double lim_min_output
  );
  // Updates the objective function error given the cross-track error
  void update_error(
      double cte
  );
  // Computes the total error for the PID controller
  double total_error();
  // Updates $\Delta t$ to the given value
  double update_delta_time(
      double new_delta_time
  );
};


#endif  //PID_CONTROLLER_H


