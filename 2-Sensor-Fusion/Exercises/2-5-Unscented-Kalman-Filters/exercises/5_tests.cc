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


/* Evaluates the result of the `generate_sigma_points` function.
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
  Xsig_expected <<
    5.7441,  5.85768,   5.7441,   5.7441,   5.7441,   5.7441,  5.63052,   5.7441,   5.7441,   5.7441,   5.7441,
    1.38,  1.34566,  1.52806,     1.38,     1.38,     1.38,  1.41434,  1.23194,     1.38,     1.38,     1.38,
    2.2049,  2.28414,  2.24557,  2.29582,   2.2049,   2.2049,  2.12566,  2.16423,  2.11398,   2.2049,   2.2049,
    0.5015,  0.44339, 0.631886, 0.516923, 0.595227,   0.5015,  0.55961, 0.371114, 0.486077, 0.407773,   0.5015,
    0.3528, 0.299973, 0.462123, 0.376339,  0.48417, 0.418721, 0.405627, 0.243477, 0.329261,  0.22143, 0.286879;
  double epsilon = 0.001;
  std::cout << "Result matches expected by amount `epsilon = " << epsilon << "`";
  std::cout << ": " << std::boolalpha << Xsig_expected.isApprox(Xsig, epsilon) << "\n";
}


/* Evaluates the result of the `augmented_sigma_points` function.
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
  std::cout << ": " << std::boolalpha << Xsig_aug_expected.isApprox(Xsig_aug, epsilon) << "\n"; 
}


int main() {
  // Exercise 2.5.1: Generating Sigma Points
  // test_generate_sigma_points();
  // Exercise 5.2.2: Generating Augmented Sigma Points
  test_augmented_sigma_points();
}