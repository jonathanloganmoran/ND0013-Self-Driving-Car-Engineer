/* ------------------------------------------------------------------------------
 * Lesson "4.2: Trajectory Generation"
 * Authors     : Sebastian Thrun, Emmanuel Boidot of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the quintic polynomial solver tests.
 *                       The functions and structs defined here are used to
 *                       evaluate the correctness of the matrix equation solver.
 * ----------------------------------------------------------------------------
 */

#ifndef GRADER_H
#define GRADER_H

#include <cmath>
#include <vector>
#include <iostream>


struct test_case {
    std::vector<double> start;
    std::vector<double> end;
    double T;
};


bool close_enough(
    std::vector<double>& poly, 
    std::vector<double>& target_poly, 
    double epsilon=0.01
) {
  if (poly.size() != target_poly.size()) {
    std::cout << "Your solution didn't have the correct number of terms" << "\n";
    return false;
  }
  for (int i = 0; i < poly.size(); ++i) {
    double diff = poly[i] - target_poly[i];
    if (abs(diff) > epsilon) {
      std::cout << "At least one of your terms differed from target by more than ";
      std::cout << epsilon << "\n";
      return false;
    }
  }
  return true;
}

std::vector<test_case> create_tests() {
  // Create test case vector
  std::vector<test_case> tc;
  // Define Test Case #1
  test_case tc1;
  tc1.start = {0, 10, 0};
  tc1.end = {10, 10, 0};
  tc1.T = 1;
  tc.push_back(tc1);
  // Define Test Case #2
  test_case tc2;
  tc2.start = {0, 10, 0};
  tc2.end = {20, 15, 20};
  tc2.T = 2;
  tc.push_back(tc2);
  // Define Test Case #3
  test_case tc3;
  tc3.start = {5, 10, 2};
  tc3.end = {-30, -20, -4};
  tc3.T = 5;
  tc.push_back(tc3);
  return tc;
}


std::vector<std::vector<double>> answers = {
    {0.0, 10.0, 0.0, 0.0, 0.0, 0.0},
    {0.0, 10.0, 0.0, 0.0, -0.625, 0.3125},
    {5.0, 10.0, 1.0, -3.0, 0.64, -0.0432}
};


#endif  // GRADER_H