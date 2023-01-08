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
 * @param  Kpi
 * @param  Kii
 * @param  Kdi
 * @param  output_lim_maxi
 * @param  output_lim_mini
 */
void PID::Init(
    double Kpi, 
    double Kii, 
    double Kdi, 
    double output_lim_maxi, 
    double output_lim_mini
) {
}

/* TODO.
 * 
 * Updates the PID errors based on the given cross-track error value.
 * 
 * @param  cte
 * 
 */
void PID::UpdateError(double cte) {
}

/* TODO.
 * 
 * Returns the total error computed w.r.t. the PID controller values.
 * 
 * @returns  control  Total computed error, should be
 *                    within range [`output_lim_maxi`, `output_lim_mini`].
 */
double PID::TotalError() {
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
double PID::UpdateDeltaTime(
    double new_delta_time
) {
}