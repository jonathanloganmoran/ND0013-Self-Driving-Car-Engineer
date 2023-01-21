/* ----------------------------------------------------------------------------
 * Lesson "5.2: Vehicle Motion Models"
 * Authors     : David Siller, Andrew Gray, Dominique Luna.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com) 
 *
 * Purpose of this file: Tests Extracurricular Exercises 5.2.1 through 5.2.2.
 * ----------------------------------------------------------------------------
 */

#include "1_global_kinematic_model.h"
#include "2_polynomial_fitting.h"



/* Evalautes the global kinematic motion model for the given values.
 * 
 * The global kinematic motion model is a simplified expression of vehicle
 * kinematics with state values defined in the global reference frame.
 * The inputs to the model are the state vector containing the vehicle position,
 * the vehicle heading and the vehicle velocity at the time-step $t$. The
 * next-state is calculated by evaluating the global kinematic model which
 * neglects the effects of gravity and any internal / external vehicle forces.
 * 
 * To evaluate the result of the global kinematic model function, we initialise
 * the starting state vector and next-state input actuations. After performing
 * the state vector update, we compare its values to the expected. If the
 * resulting values are within a margin of error given by `epsilon`, we
 * conclude that the function performs as expected.
 */
void test_global_kinematic_model() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);
  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;
  Eigen::VectorXd next_state = global_kinematic(
      state, 
      actuators, 
      0.3
  );
  /*** Checking the actual values against the expected ***/
  Eigen::VectorXd next_state_expected(4, 1);
  next_state_expected <<
    0.212132, 
    0.212132, 
    0.798488, 
    1.3;
  // Amount of L2 distance error permitted between actual and expected
  double epsilon = 0.001;
  std::cout << next_state << "\n";
  std::cout << "`next_state` matches `next_state_expected` ";
  std::cout << "(with `epsilon = " << epsilon << " ): ";
  std::cout << std::boolalpha << next_state.isApprox(
      next_state_expected, 
      epsilon
  );
  std::cout << "\n";
}


/* Tests the polynomial curve fitting functions for the given values.
 * 
 * An nth-order polynomial is fitted to the given waypoint coordinates.
 * Then, the x-coordinate values of the fitted polynomial are computed
 * by evaulating the polynomial with the original x-coordinates of the
 * waypoints.
 * 
 * This function checks the resulting coordinate values against the
 * expected. If their L2 distance differs by no more than `epsilon`,
 * we conclude that the function performs as expected.
 */
void test_polynomial_fitting() {
  // Number of points to generate for the polynomial curve 
  int n_points = 20;
  // Define the $x$- and $y$-coordinates of the waypoints
  Eigen::VectorXd xvals(6);
  Eigen::VectorXd yvals(6);
  xvals << 9.261977, -2.06803, -19.6663, -36.868, -51.6263, -66.3482;
  yvals << 5.17, -2.25, -15.306, -29.46, -42.85, -57.6116;
  // Fit the third-order polynomial on the waypoint coordinates
  Eigen::VectorXd coeffs = polyfit(xvals, yvals, 3);
  // Compute the corresponding x-coordinate values from the fitted polynomial
  Eigen::VectorXd xs(n_points + 1);
  for (double x_i = 0; x_i <= n_points; ++x_i) {
    int i = int(x_i);
    xs(i) = polyeval(coeffs, x_i);
  }
  /*** Checking the actual values against the expected ***/
  // Define the expected values from the evaluated polynomial 
  Eigen::VectorXd xs_expected(xs.size());
  xs_expected <<
    -0.905562, -0.226606, 0.447594, 1.11706, 1.7818, 2.44185, 3.09723,
    3.74794, 4.39402, 5.03548, 5.67235, 6.30463, 6.93236, 7.55555, 
    8.17423, 8.7884, 9.3981, 10.0033, 10.6041, 11.2005, 11.7925;
  // Amount of L2 distance error permitted between actual and expected
  double epsilon = 0.001;
  std::cout << xs << "\n";
  std::cout << "`xs` matches `xs_expected` ";
  std::cout << "(with `epsilon = " << epsilon << " ): ";
  std::cout << std::boolalpha << xs.isApprox(xs_expected, epsilon);
  std::cout << "\n";
}


/* Tests the Extracurricular Exercises 5.2.1 through 5.2.2.
 */
int main() {
  // Extracurricular Exercise 5.2.1: Global Kinematic Model
  test_global_kinematic_model();
  // Extracurricular Exercise 5.2.2: Polynomial Curve Fitting
  test_polynomial_fitting();
  return 0;
}