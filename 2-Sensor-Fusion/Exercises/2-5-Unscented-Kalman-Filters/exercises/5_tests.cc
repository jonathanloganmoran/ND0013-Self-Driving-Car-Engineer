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
  Eigen::MatrixXd Xsig(5, 11);
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
  Eigen::MatrixXd Xsig_aug(7, 15);
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
  Eigen::MatrixXd Xsig_aug(7, 15);
  // Generate the augmented sigma points and write them to the output matrix
  ukf.AugmentedSigmaPoints(&Xsig_aug);
  // Instantiate the output predicted sigma point matrix
  // Assumed to be of dimensions (`n_x`, `n_sigma_points`) which match the
  // values set within the `SigmaPointPrediction` function 
  Eigen::MatrixXd Xsig_pred(5, 15);
  // Compute the output matrix (the predicted state matrix)
  ukf.SigmaPointPrediction(&Xsig_pred);
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


/* Evalautes the result of the `PredictMeanAndCovariance` function.
 * 
 * The mean state estimation and covariance matrix are predicted into the next
 * time-step using the predict step equations of the Unscented Kalman Filter.
 * The prediction relies on the previous sigma point predictions from the
 * `SigmaPointPrediction` function. The resulting heading angle of the difference
 * vector between the previous and the predicted state estimation is normalised
 * to a range [-pi, pi] corresponding to the expected values for the vehicle.
 */
void test_predict_mean_and_covariance() {
  // Create the Unscented Kalman Filter (UKF) isntance
  UKF ukf;
  // Instantiate the predicted state estimation vector
  // Assumed to be of dimensions (`n_x`, 1) which match the
  // values set within the `PredictMeanAndCovariance` function 
  Eigen::VectorXd x(5, 1);
  // Instantiate the predicted covariance matrix
  // Assumed to be of dimensions (`n_x`, `n_x`) which match the
  // values set within the `PredictMeanAndCovariance` function
  Eigen::MatrixXd P(5, 5);
  // Compute the outputs (the predicted state estimation and covariance matrix)
  ukf.PredictMeanAndCovariance(
      &x,
      &P 
  );
  // Printing the resulting values
  std::cout << "x = " << "\n" << x << "\n";
  std::cout << "P = " << "\n" << P << "\n";
  // Perform the L2 norm to compare the output values
  // NOTE: these expected values hold only for the hard-coded `Xsig_pred`
  // matrix whose values are given in the Udacity VM workspace as follows:
  // Xsig_pred <<
  //        5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
  //          1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
  //         2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
  //        0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
  //         0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  Eigen::VectorXd x_expected(5, 1);
  x_expected <<
    5.93637,
    1.49035,
    2.20528,
    0.536853,
    0.353577;
  Eigen::MatrixXd P_expected(5, 5);
  P_expected <<
    0.00543425, -0.0024053,  0.00341576, -0.00348196, -0.00299378,
   -0.0024053,   0.010845,   0.0014923,   0.00980182,  0.00791091,
    0.00341576,  0.0014923,  0.00580129,  0.000778632, 0.000792973,
   -0.00348196,  0.00980182, 0.000778632, 0.0119238,   0.0112491,
   -0.00299378,  0.00791091, 0.000792973, 0.0112491,   0.0126972;
  // Precision (i.e., max allowed magnitude of the outputs' L2 difference)
  // NOTE: see above caveat (test works only when `Xsig_pred` is set manually) 
  double epsilon = 0.001;
  std::cout << "Result `x` matches expected amount by `epsilon = " << epsilon << '`';
  std::cout << ": " << std::boolalpha << x.isApprox(x_expected, epsilon) << "\n";
  std::cout << "Result `P` matches expected amount by `epsilon = " << epsilon << '`';
  std::cout << ": " << std::boolalpha << P.isApprox(P_expected, epsilon) << "\n";
}


/* Evalautes the result of the `PredictRadarMeasurement` function.
 */
void test_predict_radar_measurement() {
  // Create the Unscented Kalman Filter (UKF) isntance
  UKF ukf;
  // Instantiate the predicted state estimation vector
  // Assumed to be of dimensions (`n_z`, 1) which match the
  // values set within the `PredictRadarMeasurement` function 
  Eigen::VectorXd z_pred(3, 1);
  // Instantiate the predicted covariance matrix
  // Assumed to be of dimensions (`n_z`, `n_z`) which match the
  // values set within the `PredictRadarMeasurement` function
  Eigen::MatrixXd S(3, 3);
  // Compute the outputs (the measurement state estimation / covariance matrix)
  ukf.PredictRadarMeasurement(
      &z_pred,
      &S
  );
  // Printing the resulting values
  std::cout << "z_pred = " << "\n" << z_pred << "\n";
  std::cout << "S = " << "\n" << S << "\n";
  // Perform the L2 norm to compare the output values
  // NOTE: these expected values hold only for the hard-coded `Xsig_pred`
  // matrix whose values are given in the Udacity VM workspace as follows:
  // Xsig_pred <<
  //        5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
  //          1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
  //         2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
  //        0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
  //         0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  Eigen::VectorXd z_pred_expected(3, 1);
  z_pred_expected <<
    6.12155,
    0.245993,
    2.10313;
  Eigen::MatrixXd S_expected(3, 3);
  S_expected <<
    0.0946171,  -0.000139448,  0.00407016,
   -0.000139448, 0.000617548, -0.000770652,
    0.00407016, -0.000770652,  0.0180917;
  // Precision (i.e., max allowed magnitude of the outputs' L2 difference)
  // NOTE: see above caveat (test works only when `Xsig_pred` is set manually) 
  double epsilon = 0.001;
  std::cout << "Result `z_pred` matches expected amount by `epsilon = " << epsilon << '`';
  std::cout << ": " << std::boolalpha << z_pred.isApprox(z_pred_expected, epsilon) << "\n";
  std::cout << "Result `S` matches expected amount by `epsilon = " << epsilon << '`';
  std::cout << ": " << std::boolalpha << S.isApprox(S_expected, epsilon) << "\n";
}



int main() {
  // Exercise 2.5.1: Generating Sigma Points
  // test_generate_sigma_points();
  // Exercise 2.5.2: Generating Augmented Sigma Points
  //test_augmented_sigma_points();
  // Exercise 2.5.3: Prediction Step with Sigma Point
  // test_sigma_point_prediction();
  // Exercise 2.5.4: Prediction Step with Mean and Covariance
  // test_predict_mean_and_covariance();
  // Exercise 2.5.5: Innovation Step with Radar Measurement
  test_predict_radar_measurement();
  return 0;
}