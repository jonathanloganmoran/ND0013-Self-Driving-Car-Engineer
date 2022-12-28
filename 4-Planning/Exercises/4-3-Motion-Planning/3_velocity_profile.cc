/* ------------------------------------------------------------------------------
 * Lesson "4.3: Motion Planning"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the linear velocity profile generator.
 * ----------------------------------------------------------------------------
 */

#include "3_velocity_profile.h"


/* Returns the final speed from the given values.
 * 
 * The final speed is calculated using the expression:
 *    $v_{f} = \sqrt{v_{i}^{2} + 2 * a * d}$.
 * If the discriminant, i.e., the quantity inside the square-root,
 * is negative, then the returned final velocity $v_{f}$ is $0$.
 * If the discriminant is infinity or NaN, $v_{f} = \infty$ is returned.     
 *
 * @param    v_i    Initial velocity of the vehicle (m/s).
 * @param    a      Fixed acceleration across the given distance (m/s^2).
 * @param    d      Total distance to travel.
 * @returns  v_f    Final velocity w.r.t. given values.
 */
double calc_final_speed(
    const double v_i, 
    const double a, 
    const double d
) {
  double v_f{0.0};
  // Compute the final speed w.r.t. input values
  double discriminant = v_i * v_i + 2 * a * d;
  if (discriminant <= 0.0) {
    v_f = 0.0;
  } 
  else if (
    (discriminant == std::numeric_limits<double>::infinity()) 
    || (std::isnan(discriminant))
    ) {
    v_f = std::numeric_limits<double>::infinity();
  } 
  else {
    v_f = sqrt(discriminant);
  }
  // UNCOMMENT TO PRINT FINAL SPEED AND INITIAL VALUES
  // std::cout << "v_i, a, d: " << v_i << ", " << a << ", ";
  // std::cout << d << ",  v_f: " << v_f << "\n";
  return v_f;
}


/* Compute the distance travelled using the rectilinear acceleration equation.
 *
 * The distance travelled is computed using the rectilinear (straight-line)
 * equation for a constant acceleration `a`. This is given by the following:
 *    $\Delta{d} = \frac{v_{f}^{2} - v_{i}^{2}}{2 * a}$.
 * 
 * @param    v_i    Initial velocity of the vehicle (m/s).
 * @param    v_f    Final velocity of the vehicle (m/s).
 * @param    a      Constant acceleration of the vehicle (m/s^2).
 * @returns  d      Distance travelled w.r.t. given values.
 */
double calc_distance(
    const double v_i, 
    const double v_f, 
    const double a
) {
  if (std::abs(a) < DBL_EPSILON) {
    // Division by very small acceleration value results in infinity
    return std::numeric_limits<double>::infinity();
  }
  double d = (v_f * v_f - v_i * v_i) / (2 * a);
  // UNCOMMENT TO PRINT DISTANCE AND INITIAL VALUES
  // std::cout << "v_i, v_f, a: " << v_i << ", " << v_f << ", ";
  // std::cout << a << ",  d: " << d << "\n";
  return d;
}