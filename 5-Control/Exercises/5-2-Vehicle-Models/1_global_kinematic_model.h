/* ----------------------------------------------------------------------------
 * Lesson "5.2: Vehicle Motion Models"
 * Authors     : David Siller, Andrew Gray, Dominique Luna.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com) 
 *
 * Purpose of this file: Header file for the Global Kinematic motion model.
 * ----------------------------------------------------------------------------
 */

#include "Dense"
#include <cmath>
#include <iostream>

// Distance between the vehicle centre of mass and front axle
const double kLf;

// Helper functions
double deg2rad(
    double x
);
double rad2deg(
    double x
);

// Return the next state.
Eigen::VectorXd global_kinematic(
    const Eigen::VectorXd& state, 
    const Eigen::VectorXd& actuators, 
    double dt
);