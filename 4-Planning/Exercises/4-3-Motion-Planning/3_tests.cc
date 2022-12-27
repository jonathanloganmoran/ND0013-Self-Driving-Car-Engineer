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
#include "2_offset_goals.h"
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


/* Tests the goal-offset generator function.
 * 
 * Here the deviation from the given `offset_goal` is computed. 
 * The number of unique path deviations to compute is defined as `_num_goals`
 * inside the `2_offset_goals.h` file. The deviation for each waypoint is
 * computed with the `generate_offset_goals` function. The output of this
 * function is tested against the test cases defined here. To validate the
 * results, we check against the `expected` vector of offset waypoints.
 */
void test_generate_offset_goals(
    int argc, 
    const char* argv[]
) {
  State goal_state;
  /*** Test Case 1 ***/
  // Waypoint 1
  std::cout << "Test Case 1:" << "\n";
  goal_state.location.x = 0.0;
  goal_state.location.y = 0.0;
  goal_state.rotation.yaw = 0.0;
  State offset_goal;
  std::vector<State> expected;
  // Waypoint 2
  offset_goal.location.x = -1.83697e-16;
  offset_goal.location.y = -3;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);
  // Waypoint 3
  offset_goal.location.x = -1.22465e-16;
  offset_goal.location.y = -2;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);
  // Waypoint 4
  offset_goal.location.x = -6.12323e-17;
  offset_goal.location.y = -1;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);
  // Waypoint 5
  offset_goal.location.x = 0;
  offset_goal.location.y = 0;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);
  // Waypoint 6
  offset_goal.location.x = 6.12323e-17;
  offset_goal.location.y = 1;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);
  // Waypoint 7
  offset_goal.location.x = 1.22465e-16;
  offset_goal.location.y = 2;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);
  // Waypoint 8
  offset_goal.location.x = 1.83697e-16;
  offset_goal.location.y = 3;
  offset_goal.rotation.yaw = 0;
  expected.push_back(offset_goal);
  /*** Running the evaluation loop ***/
  // Get the goal-offset waypoints
  auto result = generate_offset_goals(goal_state);
  for (size_t i = 0; i < _num_goals; ++i) {
    std::cout << (
      (std::fabs(
        result[i].location.x - expected[i].location.x) < FLT_EPSILON
      ) && (std::fabs(
          result[i].location.y - expected[i].location.y) < FLT_EPSILON
      ) && (std::fabs(
          result[i].rotation.yaw - expected[i].rotation.yaw) < FLT_EPSILON
      ) ? "PASS" : "FAIL"
    ) << "\n";
  }
}


int main() {
  // Exercise 4.3.1: Simpson's Rule
  test_simpsons_rule();
  // Exercise 4.3.2: Goal Offset Path Planning
  test_generate_offset_goals();
  return 0;
}