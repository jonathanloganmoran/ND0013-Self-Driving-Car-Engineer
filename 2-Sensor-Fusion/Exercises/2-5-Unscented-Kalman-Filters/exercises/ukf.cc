/* ----------------------------------------------------------------------------
 * Lesson "2.5: Unscented Kalman Filters"
 * Authors     : Dominik Nuss, Andrei Vatavu.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the Unscented Kalman Filter (UKF).
 * ----------------------------------------------------------------------------
 */

#include "ukf.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;


/* Initialise the Unscented Kalman Filter instance with default values.
 */
UKF::UKF() {
  InitFilter();
}

// Destructor
UKF::~UKF() {}
// Constructor
void UKF::InitFilter() {}


/* Constructs the sigma point matrix.
 *
 * Implements the sigma point estimation for use in the prediction step.
 * The UKF performs an unscented transformation which approximates a normal
 * distribution from the non-linear posterior state estimation.
 *
 * Here the sigma points are determined with respect to the dimension of the
 * state vector and a spreading factor $\lambda$. This design parameter
 * governs how "close" a sigma point is to the mean $x_{k\vert k}$, i.e.,
 * the posterior state estimation from the previous time-step. The posterior
 * covariance matrix $\mathrm{P}_{k\vert k}$ is also used in the calculation
 * of the sigma points. The output matrix $\mathcal{x}_{k\vert k} stores
 * the resulting sigma points (`Xsig_out` in the function).
 * 
 * @param  Xsig_out   Matrix to store resulting sigma points.
 */
void UKF::GenerateSigmaPoints(
    Eigen::MatrixXd* Xsig_out
) {
  // Set the state dimension
  int n_x = 5;
  // Calculate the number of sigma points to compute
  int n_sigma_points = 2 * n_x + 1; 
  // Define the spreading parameter
  double lambda = 3 - n_x;
  // Set the state vector values
  // Here, this is assumed to be the mean of the posterior state estimation
  Eigen::VectorXd x(n_x);
  x << 5.7441,
       1.3800,
       2.2049,
       0.5015,
       0.3528;
  // Set the covariance matrix values
  // Here, this is assumed to be the covariance of posterior state estimation
  Eigen::MatrixXd P(n_x, n_x);
  P << 0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
      -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
       0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
      -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
      -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
  // Create the sigma point matrix
  Eigen::MatrixXd Xsig(n_x, n_sigma_points);
  // Calculate square-root of matrix `P`
  // `A` is the lower-triangular matrix of the Cholesky decomposition
  Eigen::MatrixXd A = P.llt().matrixL();
  /*** Calculate the set of sigma points ***/
  // Set the first column to the mean of the posterior state estimation
  Xsig.col(0) = x;
  // Compute the square-root term for the sigma point vector
  // The square-root of the spreading term
  double spreading_factor = std::sqrt(lambda + n_x);
  // Loop through the columns of `A` to compute columns of `Xsig`
  for (int i = 0; i < n_x; i++) {
    // First, update the lower column terms
    // Note the array indexing of `A` starting at 0
    Xsig.col(i + 1) = x + spreading_factor * A.col(i);
    // Then, update the upper column terms
    Xsig.col(i + n_x + 1) = x - spreading_factor * A.col(i);
  }
  // Print the resulting matrix
  // std::cout << "Xsig = " << "\n" << Xsig << "\n";
  // Write the result to the output matrix
  *Xsig_out = Xsig;
}


/* Constructs the augmented sigma point matrix. 
 *
 * Implements the sigma point estimation for the process noise $\nu_{k}$
 * used in the prediction step of the Unscented Kalman Filter.
 * 
 * Here the augmented sigma points are determined with respect to the dimension
 * of the augmented state vector $x_{a, k}$ and augmented process noise
 * covariance matrix $\mathrm{P}_{a, k\vert k}$. The mean of the process noise
 * is assumed to be zero. The augmented sigma points are computed with a design
 * parameter $\lambda$ which governs how "close" an augmented sigma point is to
 * the mean $x_{a, k\vert k}$.
 *
 * @param  Xsig_out   Matrix to store resulting augmented sigma points. 
 */
void UKF::AugmentedSigmaPoints(
    Eigen::MatrixXd* Xsig_out
) {
  // Set the state dimension
  int n_x = 5;
  // Set the augmented dimension
  // Process noise $\nu_{k}$ has the terms $\nu_{a, k}$, $\nu_{\ddot{psi},k}$
  int n_a = 2;
  // The process noise dimension added to the state vector dimension
  int n_aug = n_x + n_a;
  // Calculate the number of sigma points to compute
  int n_sigma_points = 2 * n_aug + 1;
  // Process noise standard deviation of longitudinal acceleration (m/s^2)
  double std_a = 0.2;
  // Process noise standard deviation of yaw acceleration (rad/s^2)
  double std_yawdd = 0.2;
  // Define the spreading parameter
  double lambda = 3 - n_aug;
  // Define the independent noise processes
  // Here, both are zero-mean white-noise processes
  std::normal_distribution<double> nu_a(0.0, std_a);
  std::normal_distribution<double> nu_yawdd(0.0, std_yawdd);
  // Define the noise processes vector and set its values
  std::default_random_engine rand_gen;
  Eigen::VectorXd nu_k(n_a, 1);
  nu_k << nu_a(rand_gen),
          nu_yawdd(rand_gen);
  // Set the state vector values
  // Here, this is assumed to be the mean of the posterior state estimation
  Eigen::VectorXd x(n_x);
  x << 5.7441,
       1.3800,
       2.2049,
       0.5015,
       0.3528;
  // Set the covariance matrix values
  // Here, this is assumed to be the covariance of posterior state estimation
  Eigen::MatrixXd P(n_x, n_x);
  P << 0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
      -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
       0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
      -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
      -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
  // Instantiate the augmented mean vector
  Eigen::VectorXd x_aug(n_aug);
  // Instantiate the augmented state covariance matrix
  Eigen::MatrixXd P_aug(n_aug, n_aug);
  // Instantiate the augmented sigma point matrix
  Eigen::MatrixXd Xsig_aug(n_aug, n_sigma_points);
  // Compute the values of the augmented mean state vector
  // Set the first `n_x` values to be the state vector 
  x_aug.head(n_x) = x;
  // Set the last values to be the mean of the noise processes
  x_aug.row(5) << nu_a.mean();
  x_aug.row(6) << nu_yawdd.mean(); 
  // Compute the values of the augmented covariance matrix
  Eigen::MatrixXd Q(n_a, n_a);
  Q << std_a * std_a, 0.0,
       0.0, std_yawdd * std_yawdd;
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x, n_x) = P;
  P_aug.bottomRightCorner(n_a, n_a) = Q;
  // Calcualte the square-root of the augmented covariance matrix
  // `A` is the lower-triangular matrix of the Cholesky decomposition
  Eigen::MatrixXd A_aug = P_aug.llt().matrixL();
  /*** Calculate the set of augmented sigma points ***/
  // Set the first column as mean of augmented posterior state estimation
  Xsig_aug.col(0) = x_aug;
  // Compute the square-root term for the augmented sigma point vector
  // The square-root of the spreading term
  double spreading_factor = std::sqrt(lambda + n_aug);
  // Loop through the columns of `A` to compute columns of `Xsig_aug`
  for (int i = 0; i < n_aug; i++) {
    // First, update the lower column terms
    // Note the array indexing of `A` starting at 0
    Xsig_aug.col(i + 1) = x_aug + spreading_factor * A_aug.col(i);
    // Then, update the upper column terms
    Xsig_aug.col(i + n_aug + 1) = x_aug - spreading_factor * A_aug.col(i);
  }
  // Print the resulting augmented sigma point matrix
  // std::cout << "Xsig_aug = " << "\n" << Xsig_aug << "\n";
  // Write the result to the output matrix
  *Xsig_out = Xsig_aug; 
}


/* Constructs the predicted state estimation from the augmented sigma points.
 *
 * Implements the CTRV model (i.e., Constant Turn Rate and Velocity Magnitude).
 * Here, the augmented sigma points are used to evaluate the non-linear process
 * model function $\mathcal{f}$ w.r.t. $\Delta t$.
 * 
 * The resulting point predictions are written to the right-column of the
 * predicted state estimation matrix (ensuring divide-by-zero does not occur).
 * 
 * @param  Xsig_out   Matrix to store the resulting sigma point predictions.
 */
void UKF::SigmaPointPrediction(
    Eigen::MatrixXd* Xsig_out
) {
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
  // Define the spreading parameter
  double lambda = 3 - n_aug;
  // Get the augmented sigma point matrix
  Eigen::MatrixXd Xsig_aug(n_aug, n_sigma_points);
  AugmentedSigmaPoints(&Xsig_aug);
  /*** Compute the predicted sigma point matrix ***/
  // Instantiate the predicted sigma point matrix
  Eigen::MatrixXd Xsig_pred(n_x, n_sigma_points);
  // Define the delta-time variable (s)
  double delta_t = 0.1;
  // Write the predicted sigma points into right-column of the output matrix
  for (int i = 0; i < n_sigma_points; i++) {
    // Get the state vector values from the augmented sigma point matrix
    Eigen::VectorXd x_k(n_x, 1);
    x_k << Xsig_aug.col(i).head(n_x);
    // Get the mean-values of the noise processes
    Eigen::VectorXd nu_mean_k(n_a, 1);
    // i.e., the last `n_a` values in state vector
    nu_mean_k << Xsig_aug.col(i).tail(n_a);
    // Predict the sigma point by forming the process model in state-space
    // The vector for the state transition equation 
    Eigen::VectorXd Fx_k(n_x, 1);
    // The vector of the process noise values evaluated w.r.t. time
    Eigen::VectorXd nu_k(n_x, 1);
    // Get the variables w.r.t. this sigma point
    double v_k = x_k(2);
    double yaw_k = x_k(3);
    double yawd_k = x_k(4);
    // Avoid a divide-by-zero for $\dot{\psi}$ (the yaw angle rate-of-change)
    if (yawd_k < 0.0001) {
      // Compute the state-space form of the process model
      Fx_k << 
        v_k * std::cos(yaw_k) * delta_t,
        v_k * std::sin(yaw_k) * delta_t,
        0,
        yawd_k * delta_t,
        0;
      // Compute the contribution of the process noise
      nu_k <<
        0.5 * std::pow(delta_t, 2) * std::cos(yaw_k) * nu_mean_k(0),
        0.5 * std::pow(delta_t, 2) * std::sin(yaw_k) * nu_mean_k(0),
        delta_t * nu_mean_k(0),
        0.5 * std::pow(delta_t, 2) * nu_mean_k(1),
        delta_t * nu_mean_k(1);
      // Store the sigma point prediction into the predicted state matrix
      Xsig_pred.col(i) << x_k + Fx_k + nu_k;
      continue;
    }
    // Compute the state-space form of the process model
    Fx_k <<
      (v_k / yawd_k) * (std::sin(yaw_k + yawd_k * delta_t) - std::sin(yaw_k)),
      (v_k / yawd_k) * (-std::cos(yaw_k + yawd_k * delta_t) + std::cos(yaw_k)),
      0,
      yawd_k * delta_t,
      0;
    // Compute the contribution of the process noise
    nu_k << 
      0.5 * std::pow(delta_t, 2) * std::cos(yaw_k) * nu_mean_k(0),
      0.5 * std::pow(delta_t, 2) * std::sin(yaw_k) * nu_mean_k(0),
      delta_t * nu_mean_k(0),
      0.5 * std::pow(delta_t, 2) * nu_mean_k(1),
      delta_t * nu_mean_k(1);
    // Store the sigma point prediction into the predicted state matrix
    Xsig_pred.col(i) << x_k + Fx_k + nu_k;
  }
  // Print the resulting predicted sigma point matrix
  // std::cout << "Xsig_pred = " << "\n" << Xsig_pred << "\n";
  // Write the result to the output matrix
  *Xsig_out = Xsig_pred;
}

/* Normalises the heading angle of the predicted state estimation.
 *
 * The input vector is assumed to be the difference between the predicted and
 * previous state estimation vector, s.t. the fourth row-wise element of the
 * input vector is the heading angle to normalise in range [-pi, pi].
 *
 * @param    Xsig_pred_diff   Vector of difference values between state estimates.
 * @returns  Normalised vector of the difference between state estimates. 
 */
Eigen::VectorXd NormaliseHeading(
  Eigen::VectorXd Xsig_pred_diff
) {
  while (Xsig_pred_diff(3) < -M_PI) {
    Xsig_pred_diff(3) += 2.0 * M_PI;
  }
  while (Xsig_pred_diff(3) > M_PI) {
    Xsig_pred_diff(3) -= 2.0 * M_PI;
  }
  return Xsig_pred_diff;
}


/* Constructs the predicted the mean state estimation and covariance matrix.
 *
 * The mean state vector and covariance matrix are predicted into the next
 * time-step using the sigma point prediction matrix from the previous step.
 * The predicted values are computed w.r.t. a weight vector which is defined
 * in terms of the spreading parameter $\lambda$. The weights are applied to
 * "undo" the effect of spreading on the covariance and mean state prediction.
 * 
 * @param  x_out  Predicted mean state estimation. 
 * @param  P_out  Predicted covariance matrix.
 */
void UKF::PredictMeanAndCovariance(
    Eigen::VectorXd* x_out, 
    Eigen::MatrixXd* P_out
) {
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
  // Get the augmented sigma point matrix
  Eigen::MatrixXd Xsig_aug(n_aug, n_sigma_points);
  AugmentedSigmaPoints(&Xsig_aug);
  /*** Compute the predicted sigma point matrix ***/
  // Instantiate the predicted sigma point matrix
  // NOTE: if running unit test in `5_tests.cc`, the values of the matrix
  // must be set manually using the provided definition 
  Eigen::MatrixXd Xsig_pred(n_x, n_sigma_points);
  // Define the delta-time variable (s)
  double delta_t = 0.1;
  // Get the predicted sigma points
  SigmaPointPrediction(&Xsig_pred);
  /*** Compute the predicted mean state and covariance matrix ***/
  // Instantiate the weight vector
  Eigen::VectorXd w(n_sigma_points, 1);
  // Initialise the predicted mean state vector
  Eigen::VectorXd x(n_x);
  x.fill(0.0)
  // Initialise the predicted covariance matrix
  Eigen::MatrixXd P(n_x, n_x);
  P.fill(0.0);
  // Set the weight vector values
  // Computing the first value of the weight value
  w(0) = lambda / (lambda + n_aug);
  // Computing the rest of the weight values
  for (int i = 1; i < n_sigma_points; i++) {
    w(i) = 1.0 / (2.0 * (lambda + n_aug));
  }
  // Perform the mean state estimation vector prediction
  for (int i = 0; i < n_sigma_points; i++) {
    // Compute the predicted mean state
    x += w(i) * Xsig_pred.col(i);
  }
  // Perform the covariance matrix prediction
  for (int i = 0; i < n_sigma_points; i++) {
    // Compute the difference between the state estimations
    // Then, normalise the resulting heding angle to range [-pi, pi]
    Eigen::VectorXd Xsig_pred_diff = NormaliseHeading(Xsig_pred.col(i) - x);
    // Compute the predicted covariance matrix
    P += w(i) * Xsig_pred_diff * Xsig_pred_diff.transpose();
  }
  // Print the resulting mean state and covariance matrix predictions
  std::cout << "Predicted state" << "\n";
  std::cout << x << "\n";
  std::cout << "Predicted covariance matrix" << "\n";
  std::cout << P << "\n";
  // Write the result to the input variables
  *x_out = x;
  *P_out = P;
}


/**
 * Programming assignment functions: 
 */

void UKF::PredictRadarMeasurement(VectorXd* z_out, MatrixXd* S_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // set vector for weights
  VectorXd weights = VectorXd(2*n_aug+1);
  double weight_0 = lambda/(lambda+n_aug);
  double weight = 0.5/(lambda+n_aug);
  weights(0) = weight_0;

  for (int i=1; i<2*n_aug+1; ++i) {  
    weights(i) = weight;
  }

  // radar measurement noise standard deviation radius in m
  double std_radr = 0.3;

  // radar measurement noise standard deviation angle in rad
  double std_radphi = 0.0175;

  // radar measurement noise standard deviation radius change in m/s
  double std_radrd = 0.1;

  // create example matrix with predicted sigma points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  Xsig_pred <<
         5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
           1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
          2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
         0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
          0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  // measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  /**
   * Student part begin
   */

  // transform sigma points into measurement space
  
  // calculate mean predicted measurement
  
  // calculate innovation covariance matrix S

  /**
   * Student part end
   */

  // print result
  std::cout << "z_pred: " << std::endl << z_pred << std::endl;
  std::cout << "S: " << std::endl << S << std::endl;

  // write result
  *z_out = z_pred;
  *S_out = S;
}