/* ------------------------------------------------------------------------------
 * Lesson "4.3: Motion Planning"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the linear velocity profile generator.
 * ----------------------------------------------------------------------------
 */

#include <cfloat>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>


double calc_final_speed(
    const double v_i, 
    const double a, 
    const double d
);


double calc_distance(
    const double v_i, 
    const double v_f, 
    const double a
);