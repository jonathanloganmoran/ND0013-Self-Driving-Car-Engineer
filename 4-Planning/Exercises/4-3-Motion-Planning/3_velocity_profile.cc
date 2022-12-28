/* ------------------------------------------------------------------------------
 * Lesson "4.3: Motion Planning"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the linear velocity profile generator.
 * ----------------------------------------------------------------------------
 */

/*
1) calculate the a final speed given an initial speed, an
acceleration and distance.

v_f = sqrt(v_i ^ 2 + 2ad);

2) calculate the distance traveled given an initial and final speeds and
acceleration.

d = (v_f^2 - v_i^2)/ (2 * a);
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


// ******* MAIN FUNCTION ********
int main(int argc, const char* argv[]) {
  std::cout << "Test Case 1: " << "\n";
  std::vector<double> expected_d{25.0,
                                 25.0,
                                 0.0,
                                 std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::infinity(),
                                 0.0};
  std::vector<std::vector<double>> calc_distance_input{
      {0.0, 10.0, 2.0},
      {10.0, 0.0, -2.0},
      {2.0, 2.0, 2.0},
      {2.0, std::numeric_limits<double>::infinity(), 2.0},
      {std::numeric_limits<double>::infinity(), 2.0, 3.0},
      {2.0, std::numeric_limits<double>::infinity(), 1.0},
      {12.0, 2.0, std::numeric_limits<double>::infinity()}};
  for (size_t i = 0; i < calc_distance_input.size(); ++i) {
    std::cout << (expected_d[i] == calc_distance(calc_distance_input[i][0],
                                                 calc_distance_input[i][1],
                                                 calc_distance_input[i][2])
                      ? "PASS"
                      : "FAIL")
              << "\n";
  }

  std::cout << "Test Case 2: " << "\n";
  std::vector<double> expected_vf{10.0,
                                  0.0,
                                  2.0,
                                  std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::infinity(),
                                  std::numeric_limits<double>::infinity()};
  std::vector<std::vector<double>> calc_final_speed_input{
      {0.0, 2.0, 25.0},
      {10.0, -2.0, 25.0},
      {2.0, 2.0, 0.0},
      {2.0, 0.0, std::numeric_limits<double>::infinity()},
      {std::numeric_limits<double>::infinity(), 3.0,
       std::numeric_limits<double>::infinity()},
      {2.0, 0.0, std::numeric_limits<double>::infinity()},
      {12.0, std::numeric_limits<double>::infinity(), 0.0}};
  for (size_t i = 0; i < calc_final_speed_input.size(); ++i) {
    std::cout << (expected_vf[i] ==
                          calc_final_speed(calc_final_speed_input[i][0],
                                           calc_final_speed_input[i][1],
                                           calc_final_speed_input[i][2])
                      ? "PASS"
                      : "FAIL")
              << "\n";
  }

  return 0;
}