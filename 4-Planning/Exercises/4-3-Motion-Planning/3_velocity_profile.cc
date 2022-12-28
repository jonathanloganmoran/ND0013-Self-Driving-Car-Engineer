/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: October 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/
/*
In this exercise you will practice 2 essential calculations necessary to build a
linear profile.

1) calculate the a final speed given an initial speed, an
acceleration and distance.

v_f = sqrt(v_i ^ 2 + 2ad);

2) calculate the distance traveled given an initial and final speeds and
acceleration.

d = (v_f^2 - v_i^2)/ (2 * a);
*/

#include <cfloat>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

/*
Using v_f = sqrt(v_i ^ 2 + 2ad), compute the final speed for a given
acceleration across a given distance, with initial speed v_i.
Make sure to check the discriminant of the radical.If it is negative,
return zero as the final speed.
Inputs : v_i - the initial speed in m / s.
v_f - the ginal speed in m / s.
a - the acceleration in m / s ^ 2.
*/
double calc_final_speed(const double v_i, const double a, const double d) {
  double v_f{0.0};
  // TODO-calc final speed: Calculate the final distance.
  // v_f = sqrt(v_i ^ 2 + 2ad)
  // Make sure you handle negative discriminant
  // and make v_f = 0 in that case. If the discriminant is inf or nan return
  // infinity.
  double disc = ...;  // <-- FIX THIS
  if (disc <= 0.0) {
    v_f = ...;  // <-- FIX THIS
  } else if (disc == std::numeric_limits<double>::infinity() ||
             std::isnan(disc)) {
    v_f = ...;  // <-- FIX THIS
  } else {
    v_f = ...;  // <-- FIX THIS
  }
  //   std::cout << "v_i, a, d: " << v_i << ", " << a << ", " << d
  //             << ",  v_f: " << v_f << std::endl;
  return v_f;
}

/*
Using d = (v_f^2 - v_i^2) / (2 * a), compute the distance
required for a given acceleration/deceleration.

Inputs: v_i - the initial speed in m/s.
        v_f - the final speed in m/s.
        a - the acceleration in m/s^2.
        */
double calc_distance(const double v_i, const double v_f, const double a) {
  double d{0.0};
  // TODO-calc distance: use one of the common rectilinear accelerated
  // equations of motion to calculate the distance traveled while going from
  // v_i (initial velocity) to v_f (final velocity) at a constant
  // acceleration/deceleration "a".
  // d = (v_f^2 - v_i^2) / (2 * a)
  // Make sure you handle div by 0 (if (std::abs(a) < DBL_EPSILON))

  // YOUR CODE HERE

  //   std::cout << "v_i, v_f, a: " << v_i << ", " << v_f << ", " << a
  //             << ",  d: " << d << std::endl;
  return d;
}

// ******* MAIN FUNCTION ********
int main(int argc, const char* argv[]) {
  std::cout << "Test Case 1: " << std::endl;
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
              << std::endl;
  }

  std::cout << "Test Case 2: " << std::endl;
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
              << std::endl;
  }

  return 0;
}