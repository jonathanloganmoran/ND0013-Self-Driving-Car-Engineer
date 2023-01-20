/* ----------------------------------------------------------------------------
 * Lesson "5.2: Vehicle Motion Models"
 * Authors     : David Siller, Andrew Gray, Dominique Luna.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com) 
 *
 * Purpose of this file: Header file for the polynomial curve fitting function.
 * ----------------------------------------------------------------------------
 */

#include "Dense"          // `Eigen::VectorXd`, `householderQr()`
#include <iostream>


// Evaluates the `y` coordinates of the given polynomial 
double polyeval(
    const Eigen::VectorXd& coeffs, 
    double x
);


// Fits a polynomial curve to the given waypoints.
Eigen::VectorXd polyfit(
    const Eigen::VectorXd& xvals, 
    const Eigen::VectorXd& yvals, 
    int order
);