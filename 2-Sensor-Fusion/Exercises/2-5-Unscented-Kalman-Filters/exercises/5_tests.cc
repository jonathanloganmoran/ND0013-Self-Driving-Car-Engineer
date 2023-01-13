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
  // NOTE: must set the `Xsig_pred` matrix to the values defined below in order
  // to obtain output `z_pred` and `S` with corresponding expected values
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
  double epsilon = 0.0001;
  std::cout << "Result `z_pred` matches expected amount by `epsilon = " << epsilon << '`';
  std::cout << ": " << std::boolalpha << z_pred.isApprox(z_pred_expected, epsilon) << "\n";
  std::cout << "Result `S` matches expected amount by `epsilon = " << epsilon << '`';
  std::cout << ": " << std::boolalpha << S.isApprox(S_expected, epsilon) << "\n";
}


void test_update_state() {
  // Create the Unscented Kalman Filter (UKF) instance
  UKF ukf;
  /*** Set the state variables (values should match across functions) ***/
  // Set the state dimension
  int n_x = 5;
  // Set the augmented dimension
  // Process noise $\nu_{k}$ has the terms $\nu_{a, k}$, $\nu_{\ddot{psi},k}$
  int n_a = 2;
  // The process noise dimension added to the state vector dimension
  int n_aug = n_x + n_a;
  // Calculate the number of sigma points to compute
  int n_sigma_points = 2 * n_aug + 1;
  // Set the measurement dimensions
  // Note: the radar measurement vector is $[\rho, \phi, \dot{\rho}]$
  // i.e., the measured radial distance, angle, and radial velocity
  int n_z = 3;
  // Define the spreading parameter
  double lambda = 3 - n_aug;
  // Define the delta-time variable (s)
  double delta_t = 0.1;
  // Define the incoming radar measurement vector
  // assumed to be the same in `UpdateState`
  Eigen::VectorXd z(n_z);
  z <<
     5.9214,   // rho in m
     0.2187,   // phi in rad
     2.0062;   // rho_dot in m/s
  // Instantiate the updated state estimation
  Eigen::VectorXd x(n_x, 1);
  // Instantiate the updated covariance matrix
  Eigen::MatrixXd P(n_x, n_x);
  /*** Define the variables needed for the test case ***/
  // NOTE: the following values must be set inside `UpdateState`
  // in order for the expected test case values to hold
  // Eigen::MatrixXd Xsig_pred(n_x, n_sigma_points);
  // Xsig_pred <<
  //    5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
  //      1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
  //     2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
  //    0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
  //     0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  // Eigen::VectorXd x(n_x);
  // x <<
  //    5.93637,
  //    1.49035,
  //    2.20528,
  //   0.536853,
  //   0.353577;
  // Eigen::MatrixXd P(n_x, n_x);
  // P <<
  //   0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
  //   -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
  //   0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
  //  -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
  //  -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;
  // Eigen::MatrixXd Zsig(n_z, n_sigma_points);
  // Zsig <<
  //   6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
  //  0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
  //   2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;
  // Eigen::VectorXd z_pred(n_z);
  // z_pred <<
  //   6.12155,
  //    0.245993,
  //    2.10313;
  // Eigen::MatrixXd S(n_z, n_z);
  // S <<
  //     0.0946171, -0.000139448,   0.00407016,
  //  -0.000139448,  0.000617548, -0.000770652,
  //    0.00407016, -0.000770652,    0.0180917;
  /*** Perform the UKF Innovation step ***/
  ukf.UpdateState(&x, &P);
  // Perform the L2 norm to compare the output values
  Eigen::VectorXd x_expected(5, 1);
  x_expected <<
    5.92276,
    1.41823,
    2.15593,
    0.489274,
    0.321338;
  Eigen::MatrixXd P_expected(5, 5);
  P_expected <<
    0.00361579, -0.000357881, 0.00208316, -0.000937196, -0.00071727,
   -0.000357881, 0.00539867,  0.00156846,  0.00455342,   0.00358885,
    0.00208316,  0.00156846,  0.00410651,  0.00160333,   0.00171811,
   -0.000937196, 0.00455342,  0.00160333,  0.00652634,   0.00669436,
   -0.00071719,  0.00358884,  0.00171811,  0.00669426,   0.00881797;
  // Precision (i.e., max allowed magnitude of the outputs' L2 difference)
  // NOTE: test works only when all variables above are set manually 
  double epsilon = 0.0001;
  std::cout << "Result `x` matches expected amount by `epsilon = " << epsilon << '`';
  std::cout << ": " << std::boolalpha << x.isApprox(x_expected, epsilon) << "\n";
  std::cout << "Result `P` matches expected amount by `epsilon = " << epsilon << '`';
  std::cout << ": " << std::boolalpha << P.isApprox(P_expected, epsilon) << "\n";
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
  // test_predict_radar_measurement();
  // Exercise 2.5.6: Innovation Step with State and Covariance Update
  test_update_state();
  return 0;
}