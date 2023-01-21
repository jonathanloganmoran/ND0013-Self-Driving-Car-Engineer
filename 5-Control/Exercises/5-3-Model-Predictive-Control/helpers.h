/* ----------------------------------------------------------------------------
 * Lesson "5.3: Model Predictive Control"
 * Authors     : David Siller, Andrew Gray, Dominique Luna.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com) 
 *
 * Purpose of this file: Header file for the polynomial curve fitting function.
 * ----------------------------------------------------------------------------
 */

#ifndef HELPERS_H
#define HELPERS_H

#include "include/eigen-3.3.7/Eigen/QR"         // `householderQr()`


/* Evaluates the `y` coordinates of a given polynomial.
 *
 * @param    coeffs   Coefficients of the fitted polynomial.
 * @param    x        Coordinates of the waypoints along the $x$-axis.
 * @returns  result   The corresponding coordinates along the $y$-axis.
 */
double polyeval(
    const Eigen::VectorXd& coeffs, 
    double x
) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); ++i) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}


/* Fits a polynomial curve to the given waypoints.
 *
 * Adapted from:
 * https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
 * 
 * @param    yvals    Coordinates of the waypoints along the $y$-axis.
 * @param    xvals    Coordinates of the waypoints along the $x$-axis.
 * @param    order    Order of the polynomial to fit.
 * @returns  result   The vector of coefficients of the fitted polynomial.
 */
Eigen::VectorXd polyfit(
    const Eigen::VectorXd& xvals, 
    const Eigen::VectorXd& yvals, 
    int order
) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);
  for (int i = 0; i < xvals.size(); ++i) {
    A(i, 0) = 1.0;
  }
  for (int j = 0; j < xvals.size(); ++j) {
    for (int i = 0; i < order; ++i) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }
  auto Q = A.householderQr();
  Eigen::VectorXd result = Q.solve(yvals);
  return result;
}

#endif  // HELPERS_H