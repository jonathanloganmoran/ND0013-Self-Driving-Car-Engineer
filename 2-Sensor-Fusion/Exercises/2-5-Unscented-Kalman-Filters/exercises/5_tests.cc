/* ----------------------------------------------------------------------------
 * Lesson "2.5: Unscented Kalman Filters"
 * Authors     : Dominik Nuss, Andrei Vatavu.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Tests files from Exercises 2.5.1 through 2.5..
 * ----------------------------------------------------------------------------
 */

#include "ukf.h"
#include "Dense"                    // The Eigen matrix manipulation library
#include <iostream>


/* Evaluates the result of the `GenerateSigmaPoints` function.
 *
 * The actual sigma points produced by the function are compared to the
 * expected matrix of sigma point values by computing the Frobenius L2
 * norm (matrix norm). If the resulting L2 norm value is less than the
 * `epsilon` threshold, the two matrices are said to be roughly equal.
 */
void test_generate_sigma_points() {
  // Create the Unscented Kalman Filter (UKF) instance
  UKF ukf;
  // Instantiate the sigma point output matrix 
  MatrixXd Xsig = MatrixXd(5, 11);
  // Generate the sigma points and write them to the output matrix
  ukf.GenerateSigmaPoints(&Xsig);
  // Print the resulting values
  std::cout << "Xsig = " << "\n" << Xsig << "\n";
  // Perform L2 norm to compare the two matrices
  Eigen::MatrixXd Xsig_expected(5, 11);
  Xsig_expected <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441,
    1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879;
  double epsilon = 0.001;
  std::cout << "Result matches expected by amount `epsilon = " << epsilon << "`";
  std::cout << ": " << std::boolalpha << Xsig.isApprox(Xsig_expected, epsilon) << "\n";
}


/* Evaluates the result of the `AugmentedSigmaPoints` function.
 *
 * The actual augmented sigma points produced by the function are compared
 * to the expected matrix of augmented sigma point values by computing the
 * Frobenius L2 norm (matrix norm). If the resulting L2 norm value is less
 * than the `epsilon` threshold, the two matrices are said to be roughly
 * equal.
 */
void test_augmented_sigma_points() {
  // Create the Unscented Kalman Filter (UKF) instance
  UKF ukf;
  // Instantiate the augmented sigma point matrix
  // Note: We assume the dimension of the augmented state vector to be `7`,
  // and the number of augmented sigma points generated to be `15`
  Eigen::MatrixXd(7, 15) Xsig_aug;
  // Generate the augmented sigma points and write them to the output matrix
  ukf.AugmentedSigmaPoints(&Xsig_aug);
  // Print the resulting values
  std::cout << "Xsig_aug = " << "\n" << Xsig_aug << "\n";
  // Perform the L2 norm to compare the two matrices
  Eigen::MatrixXd Xsig_aug_expected(7, 15);
  Xsig_aug_expected <<
  5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,   5.7441,
    1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,     1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38,     1.38,     1.38,
  2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049,   2.2049,   2.2049,
  0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,   0.5015,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015,   0.5015,   0.5015,
  0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721,   0.3528,   0.3528, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879,   0.3528,   0.3528,
       0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0,        0, -0.34641,        0,
       0,        0,        0,        0,        0,        0,        0,  0.34641,        0,        0,        0,        0,        0,        0, -0.34641;
  // Precision (i.e., max allowed magnitude of the two matrices' L2 distance)
  double epsilon = 0.001;
  std::cout << "Result matches expected by amount `epsilon = " << epsilon << "`";
  std::cout << ": " << std::boolalpha << Xsig_aug.isApprox(Xsig_aug_expected, epsilon) << "\n"; 
}


/* Evaluates the result of the `SigmaPointPrediction` function.
 *
 * The state transition function of the CTRV model is evaluated using the
 * augmented sigma points computed with the `AugmentedSigmaPoints` function. 
 * The state transition function is given in state-space form and is defined
 * with respect to the $\Delta t$ parameter. The output of this function, i.e.,
 * the resulting point predictions, are written to the predicted state
 * estimation matrix `Xsig_out`. The values of this output matrix are compared
 * to the expected values defined here using the Frobenius L2 norm. If the
 * resulting L2 norm value is less than the `epsilon` threshold, the two
 * matrices are said to be roughly equal.
 */
void test_sigma_point_prediction() {
  // Create the Unscented Kalman Filter (UKF) instance
  UKF ukf;
  // Instantiate the augmented sigma point matrix
  // Assumed to be of dimensions (`n_aug`, `n_sigma_points`) which match the
  // values set within the `AugmentSigmaPoints` function
  Eigen::MatrixXd(7, 15) Xsig_aug;
  // Generate the augmented sigma points and write them to the output matrix
  ukf.AugmentSigmaPoints(&Xsig_aug)
  // Instantiate the output predicted sigma point matrix
  // Assumed to be of dimensions (`n_x`, `n_sigma_points`) which match the
  // values set within the `SigmaPointPrediction` function 
  Eigen::MatrixXd(5, 15) Xsig_pred;
  // Compute the output matrix (the predicted state matrix)
  SigmaPointPrediction(&Xsig_pred);
  // Print the resulting values
  std::cout << "Xsig_pred = " << Xsig_pred << "\n";
  // Perform the L2 norm to compare the two matrices
  Eigen::MatrixXd Xsig_pred_expected(5, 15);
  Xsig_pred_expected <<
    5.93553, 6.06251,  5.92217,  5.9415,   5.92361,  5.93516,  5.93705, 5.93553,  5.80832,  5.94481,  5.92935,  5.94553,  5.93589,  5.93401, 5.93553,
    1.48939, 1.44673,  1.66484,  1.49719,  1.508,    1.49001,  1.49022, 1.48939,  1.5308,   1.31287,  1.48182,  1.46967,  1.48876,  1.48855, 1.48939,
    2.2049,  2.28414,  2.24557,  2.29582,  2.2049,   2.2049,   2.23954, 2.2049,   2.12566,  2.16423,  2.11398,  2.2049,   2.2049,   2.17026, 2.2049,
    0.53678, 0.473387, 0.678098, 0.554557, 0.643644, 0.543372, 0.53678, 0.538512, 0.600173, 0.395462, 0.519003, 0.429916, 0.530188, 0.53678, 0.535048,
    0.3528,  0.299973, 0.462123, 0.376339, 0.48417,  0.418721, 0.3528,  0.387441, 0.405627, 0.243477, 0.329261, 0.22143,  0.286879, 0.3528, 0.318159;
  // Precision (i.e., max allowed magnitude of the two matrices' L2 distance)
  double epsilon = 0.001;
  std::cout << "Result matches expected by amount `epsilon = " << epsilon << "`";
  std::cout << ": " << std::boolalpha << Xsig_pred.isApprox(Xsig_pred_expected, epsilon) << "\n";
}


int main() {
  // Exercise 2.5.1: Generating Sigma Points
  // test_generate_sigma_points();
  // Exercise 2.5.2: Generating Augmented Sigma Points
  test_augmented_sigma_points();
}