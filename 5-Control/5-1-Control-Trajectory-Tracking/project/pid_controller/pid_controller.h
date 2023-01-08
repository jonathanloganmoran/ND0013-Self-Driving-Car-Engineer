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
 * @var  
 */
class PID {
 public:
  /**
  * TODO: Create the PID class
  **/
  /*
   * Errors
   */
  // The cross-track error value from the previous time-step
  double cte_prev;
  // The cross-track error value from the current time-step
  double cte_curr;
  // The cumulative cross-track error across all time-steps `n`
  double cte_total;
  /*
   * Coefficients
   */
  double k_p;
  double k_i;
  double k_d;
  /*
   * Output limits
   */
  double lim_max_output;
  double lim_min_output;
  /*
   * Delta time
   */
  double delta_t;

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
  // Updates the objective functino error given the cross-track error
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


