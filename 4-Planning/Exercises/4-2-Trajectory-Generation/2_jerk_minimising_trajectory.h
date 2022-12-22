/* ------------------------------------------------------------------------------
 * Lesson "4.2: Trajectory Generation"
 * Authors     : Sebastian Thrun, Emmanuel Boidot of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the quintic polynomial solver. This
 *                       solver is used to generate jerk-minimised trajectories.
 * ----------------------------------------------------------------------------
 */


#include "grader.h"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>


// Returns the coefficients of the quintic polynomial
std::vector<double> JMT(
    std::vector<double>& start,
    std::vector<double>& end,
    double T
);