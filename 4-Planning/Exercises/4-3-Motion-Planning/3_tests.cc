/* ------------------------------------------------------------------------------
 * Lesson "4.3: Motion Planning"
 * Authors     : Munir Jojo-Verge.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Tests for Exercises 4.3.1 - 4.3.3.
 * ----------------------------------------------------------------------------
 */

#include "1_simpsons_rule.h"
#include <iostream>
#include <vector>


/* Tests the Simpson's 1/3 rule for numerical approximation of a cubic sprial.
 *
 * Assumed here in the equation for $\theta(s)$ is $\theta_{0} = 0$, which
 * results in the following expression for the cubic sprial:
 *    $\theta(s) = \frac{a_{3}s^{4}}{4} + \frac{a_{2}s^{3}}{2} + a_{0}s$.
 */
void test_simpsons_rule() {
  // Length of the cubic spiral
  double sg = 10.0;
  // Number of subintervals to divide the integration limits for Simpson's rule
  size_t n = 9;
  double delta_s = sg / n;
  // Define the coefficients of the cubic spiral,
  // i.e., K(s)= a3*s^3 + a2*s^2 + a1*s + a0, s.t. `a=[a0, a1, a2, a3]`.
  std::array<double, 4> a{0.0, 0.045, 0.0225, -0.0027};
  // Compute the integrand,
  // i.e., the function to integrate: f(s) = cos(theta(s)).
  std::vector<double> f_s0_sn = generate_f_s0_sn(delta_s, a, n);
  double integral_result = IntegrateBySimpson(f_s0_sn, delta_s, n);
  // Define the expected result of the integral approximation
  double expected = 2.3391428316;
  // Check if the result is within a given `epsilon` of the expected
  double epsilon =  0.00001;
  std::cout << (
    fabs(expected - integral_result) < epsilon ? "PASS" : "FAIL"
  ) << "\n";
  std::cout << "Result: " << integral_result << "\n";
}


int main() {
  // Exercise 4.3.1: Simpson's Rule
  test_simpsons_rule();
  return 0;
}