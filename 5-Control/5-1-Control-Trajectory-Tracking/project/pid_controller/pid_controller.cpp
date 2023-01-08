/* ------------------------------------------------------------------------------
 * Project "5.1: Control and Trajectory Tracking for Autonomous Vehicles"
 * Authors     : Mathilde Badoual.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the PID controller.
 * ----------------------------------------------------------------------------
 */

#include "pid_controller.h"


PID::PID() {}
PID::~PID() {}


/* Initialises the PID controller parameters with the given values.
 *
 * @param  k_p             Gain value to use for the proportional term.
 * @param  k_i             Gain value to use for the integral term.
 * @param  k_d             Gain value to use for the derivative term.
 * @param  lim_max_output  Maximum output value (used to threshold controller).
 * @param  lim_min_output  Minimum output value (used to threshold controller).
 */
void PID::init_controller(
    double k_p, 
    double k_i, 
    double k_d, 
    double lim_max_output, 
    double lim_min_output
) {
  this->k_p = k_p;
  this->k_i = k_i;
  this->k_d = k_d;
  this->lim_max_output = lim_max_output;
  this->lim_min_output = lim_min_output;
  this->delta_t = 0.0;
  this->error_p = 0.0;
  this->error_i = 0.0;
  this->error_d = 0.0;
}


/* Evaluates the PID controller expression using the given cross-track error.
 * 
 * @param  cte    Current cross-track error value. 
 * 
 */
void PID::update_error(
    double cte
) {
  // Update the proportional-gain error
  this->error_p = cte;
  // Update the derivative-gain error term
  if (this->delta_t > 0.0) {
    // The $\dot{cte}$ term, i.e., the derivative of CTE w.r.t. $\Delta t$
    this->error_d = (cte - error_p) / this->delta_t;
  }
  else {
    // Divide by zero: set resulting derivative error to 0.0
    this->error_d = 0.0; 
  }
  // Update the integral-gain error term
  this->error_i += cte * this->delta_t;
}


/* TODO.
 * 
 * Returns the total error computed w.r.t. the PID controller values.
 * 
 * @returns  control  Total computed error, should be within the range
 *                    [`lim_max_output`, `lim_min_output`].
 */
double PID::total_error() {
    double control;
    return control;
}


/* TODO.
 *
 * Updates $\Delta t$ to the given value.
 * 
 * @param    new_delta_time  Elapsed time interval (s) value to set.
 * @returns  delta_time      Updated time interval (s).
 */
double PID::update_delta_time(
    double new_delta_time
) {
}