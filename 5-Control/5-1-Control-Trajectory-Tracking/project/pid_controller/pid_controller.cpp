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


/* TODO.
 *
 * Initialises the PID controller parameters with the given values.
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
}


/* TODO.
 * 
 * Updates the PID errors based on the given cross-track error value.
 * 
 * @param  cte
 * 
 */
void PID::update_error(
    double cte
) {
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