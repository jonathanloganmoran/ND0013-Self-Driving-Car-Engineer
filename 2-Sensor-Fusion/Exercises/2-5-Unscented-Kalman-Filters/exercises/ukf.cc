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
  // Update the input pointer to the output result
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
  // Update the input pointer to the output result
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
  // Update the input pointer to the output result
  *Xsig_out = Xsig_pred;
}


namespace SigmaPoints {
/* Normalises the heading angle of the predicted state estimation.
 *
 * The input vector is assumed to be the difference between the predicted and
 * previous state estimation vector, s.t. the fourth row-wise element of the
 * input vector is the heading angle to normalise in range [-pi, pi].
 *
 * @param    x_diff   Vector of difference values between state estimates.
 * @returns  Normalised vector of the difference between state estimates. 
 */
Eigen::VectorXd NormaliseHeading(
  Eigen::VectorXd x_diff
) {
  while (x_diff(3) < -M_PI) {
    x_diff(3) += 2.0 * M_PI;
  }
  while (x_diff(3) > M_PI) {
    x_diff(3) -= 2.0 * M_PI;
  }
  return x_diff;
}
}  // namespace SigmaPoints


namespace Radar {
/* Normalises the heading angle of the radar measurement state estimatation.
 *
 * The input vector is assumed to be the radar measurement vector s.t. the
 * second row-wise element of the input vector is the heading angle to
 * normalise in range [0, pi].
 *
 * @param    z_diff   Vector of difference values between radar measurements.
 * @returns  Normalised vector of the difference in radar measurements.
 */
Eigen::VectorXd NormaliseHeading(
  Eigen::VectorXd z_diff
) {
  while (z_diff(1) < -M_PI) {
    z_diff(1) += 2.0 * M_PI;
  }
  while (z_diff(1) > M_PI) {
    z_diff(1) -= 2.0 * M_PI;
  }
  return z_diff;
}
} // namespace Radar


/* Constructs the predicted the mean state estimation and covariance matrix.
 *
 * The mean state vector and covariance matrix are predicted into the next
 * time-step using the sigma point prediction matrix from the previous step.
 * The predicted values are computed w.r.t. a weight vector which is defined
 * in terms of the spreading parameter $\lambda$. The weights are applied to
 * "undo" the effect of spreading on the covariance and mean state prediction.
 * 
 * @param  x_out  Vector to store predicted mean state estimation. 
 * @param  P_out  Matrix to store predicted covariance.
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
    // Then, normalise the resulting heading angle to range [-pi, pi]
    Eigen::VectorXd Xsig_pred_diff = (
      SigmaPoints::NormaliseHeading(Xsig_pred.col(i) - x)
    );
    // Compute the predicted covariance matrix
    P += w(i) * Xsig_pred_diff * Xsig_pred_diff.transpose();
  }
  // Print the resulting mean state and covariance matrix predictions
  std::cout << "Predicted state" << "\n";
  std::cout << x << "\n";
  std::cout << "Predicted covariance matrix" << "\n";
  std::cout << P << "\n";
  // Update the input pointers to the output results
  *x_out = x;
  *P_out = P;
}


/* Constructs the radar measurement mean estimation and covariance matrix.
 * 
 * The radar measurement vector is a three-dimensional vector:
 *   $z_{k + 1\vert k} = [\rho, \phi, \dot{\rho}]$,
 * where $\rho$ is the radial distance (m), $\phi$ is the angle (rad), and
 * $\dot{\rho}$ is the radial velocity (m/s) measured by the radar sensor.
 * 
 * In order to perform the update / innovation step, several "shortcuts" are
 * used. The first involves the "recycling" of the sigma point matrix, which,
 * due to the purely additive (linear) contribution of the radar measurement
 * noise, allows us to skip the computation of new sigma points and the UKF
 * augmentation step all-together. The second "shortcut" used here the hold-out
 * of the measurement noise $\omega_{k+1}$ from the measurement model function.
 * Again, due to the linearity of the noise contribution, we neglect the noise
 * value until the computation of the measurement covariance prediction matrix
 * $\mathrm{R}$ in the innovation / correction step.
 * 
 * The equations for the sigma point-to-radar measurement space are given by:
 *    $\rho = \sqrt{p_{x}^{2} + p_{y}^{2}}$,
 *    $\phi = \arctan(p_{y} / p_{x})$,
 *    $$\begin{align}
 *    \dot{\rho} &= \frac{
 *        p_{x}\cos(\phi)*v + p_{y}\sin(\phi)*v
 *        }{\sqrt{p_{x}^{2} + p_{y}^{2}}} \\
 *    \end{align}$$.
 * 
 * @param  z_out  Vector to store predicted measurement mean state estimation.
 * @param  S_out  Matrix to store predicted measurement covariance.
 */
void UKF::PredictRadarMeasurement(
    Eigen::VectorXd* z_out, 
    Eigen::MatrixXd* S_out
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
  // Set the measurement dimensions
  // Note: the radar measurement vector is $[\rho, \phi, \dot{\rho}]$
  // i.e., the measured radial distance, angle, and radial velocity
  int n_z = 3;
  // Define the spreading parameter
  double lambda = 3 - n_aug;
  // Set the weight vector values
  Eigen::VectorXd w(n_sigma_points, 1);
  w(0) = lambda / (lambda + n_aug);
  double weight = 1.0 / (2.0 * (lambda + n_aug));
  for (int i = 1; i < n_sigma_points; ++i) {  
    w(i) = weight;
  }
  // Standard deviation of the radar measurement noise for $\rho$ (m)
  double std_radr = 0.3;
  // Standard deviation of the radar measurment noise for $\phi$ (rad)
  double std_radphi = 0.0175;
  // Standard deviation of the radar measurment noise for $\dot{\rho}$ (m/s)
  double std_radrd = 0.1;
  // Define the measurement noise covariance $\mathrm{R}$
  Eigen::MatrixXd R(n_z, n_z);
  // Set the measurement noise covariance matrix values
  R << std_radr * std_radr, 0.0, 0.0,
       0.0, std_radphi * std_radphi, 0.0,
       0.0, 0.0, std_radrd * std_radrd;
  // Get the predicted sigma point matrix
  Eigen::MatrixXd Xsig_pred(n_x, 2 * n_aug + 1);
  SigmaPointPrediction(&xsig_pred);
  // Initialise the sigma point matrix in measurement space
  Eigen::MatrixXd Zsig(n_z, n_sigma_points);
  Zsig.fill(0.0);
  // Initialise the mean predicted measurement vector
  Eigen::VectorXd z_pred(n_z);
  z_pred.fill(0.0);
  // Initialise the measurement covariance matrix S
  Eigen::MatrixXd S(n_z, n_z);
  S.fill(0.0);
  // Transform the sigma points into the radar measurement space
  for (int i = 0; i < n_sigma_points; i++) {
    // Get the state vector associated with the sigma point
    Eigen::VectorXd x_i = Xsig_pred.col(i);
    // Store the transformed state vector in measurement space
    Eigen::VectorXd Zsig_i(n_z, 1);
    // Compute the radial distance $\rho$ transformation
    Zsig_i(0) = std::sqrt(std::pow(x_i(0), 2) + std::pow(x_i(1), 2));
    // Compute the angle $\phi$ transformation
    Zsig_i(1) = std::atan(x_i(1) / x_i(0));
    // Compute the radial velocity $\dot{\rho}$ transformation
    Zsig_i(2) = (
      x_i(0) * std::cos(x_i(3)) * x_i(2) + x_i(1) * std::sin(x_i(3)) * x_i(2)
    ) / std::sqrt(std::pow(x_i(0), 2) + std::pow(x_i(1), 2));
    // Update the resulting sigma point in measurment space vector
    Zsig.col(i) = Zsig_i;
  }
  // Calculate the mean predicted measurement vector
  for (int i = 0; i < n_sigma_points; i++) {
    // Compute the `i`th predicteded measurement mean $z_{k+1 \vert k}$
    z_pred += w(i) * Zsig.col(i);
  }
  // Calculate the covariance matrix `S` of the innovation / correction step
  for (int i = 0; i < n_sigma_points; i++) {
    // Compute the difference in measurement mean state estimations
    // Then, normalise the resulting heading angle estimate to range [-pi, pi]
    Eigen::VectorXd Zsig_meas_diff = (
      Radar::NormaliseHeading(Zsig.col(i) - z_pred)
    );
    // Compute the `i`th covariance matrix $S_{k+1\vert k}$ step
    S += w(i) * Zsig_meas_diff * Zsig_meas_diff.transpose();
  }
  // Set the additive contribution of the measurement noise to the vector
  // as defined by the measurement noise covariance matrix $\mathrm{R}$
  S += R;
  // Print the resulting outputs
  std::cout << "z_pred: " << "\n" << z_pred << "\n";
  std::cout << "S: " << "\n" << S << "\n";
  // Update the input pointers to the output results
  *z_out = z_pred;
  *S_out = S;
}


/* Performs the UKF Innovation step to update the state mean and covariance.
 * 
 * The Innovation step of the Unscented Kalman Filter (UKF) differs from
 * the standard Kalman filter (KF) only by a cross-correlation term
 * computed between the state- and measurement space of the sigma points.
 * This term is used in the equation for the Kalman gain given by:
 * $$\begin{align}
 * \mathrm{K}_{k+1\vert k} &= \mathrm{T}_{k+1\vert k} 
 *                            * \mathrm{S}^{\inv}_{k+1\vert k}. 
 * \end{align}$$
 * 
 * The state update equation for the time-step $k+1$ is given by:
 * $$\begin{align}
 * x_{k+1\vert k+1} = x_{k+1\vert k}
 *                    + \mathrm{K}_{k+1\vert k}
 *                    * (z_{k+1} - z_{k+1\vert k}).
 * \end{align}$$
 * 
 * The covariance matrix update equation for time-step $k+1$ is given by:
 * $$\begin{align}
 * \mathrm{P}_{k+1\vert k+1} = \mathrm{P}_{k+1\vert k}
 *                             - \mathrm{K}_{k+1 \ k}
 *                             * \mathrm{S}_{k+1 \vert k}
 *                             * \mathrm{K}^{\top}_{k+1 \vert k}.
 * \end{align}$$
 * 
 * The cross-correlation matrix between the sigma points in state- and
 * measurement-space is given as:
 * $$\begin{align}
 * \mathrm{T}_{k+1\vert k} = 
 * \sum_{i=0}^{2*n_{a}} w_{i} 
 *                      * (\mathcal{x}_{k+1\vert k,i} - x_{k+1\vert k})
 *                      * (\mathcal{z}_{k+1\vert k,i} - z_{k+1\vert k})^{\top}.
 * \end{align}$$
 * 
 * @param  x_out    Vector to store the updated mean state estimate. 
 * @param  P_out    Matrix to store the updated covariance matrix estimate.
 */
void UKF::UpdateState(
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
  // Set the measurement dimensions
  // Note: the radar measurement vector is $[\rho, \phi, \dot{\rho}]$
  // i.e., the measured radial distance, angle, and radial velocity
  int n_z = 3;
  // Define the spreading parameter
  double lambda = 3 - n_aug;
  // Set the weight vector values
  Eigen::VectorXd w(n_sigma_points, 1);
  w(0) = lambda / (lambda + n_aug);
  double weight = 1.0 / (2.0 * (lambda + n_aug));
  for (int i = 1; i < n_sigma_points; ++i) {  
    w(i) = weight;
  }
  /*** Compute the predicted sigma point matrix ***/
  // Define the delta-time variable (s)
  double delta_t = 0.1;
  // Instantiate the predicted sigma point matrix
  // NOTE: if running unit test in `5_tests.cc`, the values of the matrix
  // must be set manually using the provided definition
  Eigen::MatrixXd Xsig_pred(n_x, n_sigma_points);
  // Xsig_pred <<
  //    5.9374,  6.0640,   5.925,  5.9436,  5.9266,  5.9374,  5.9389,  5.9374,  5.8106,  5.9457,  5.9310,  5.9465,  5.9374,  5.9359,  5.93744,
  //      1.48,  1.4436,   1.660,  1.4934,  1.5036,    1.48,  1.4868,    1.48,  1.5271,  1.3104,  1.4787,  1.4674,    1.48,  1.4851,    1.486,
  //     2.204,  2.2841,  2.2455,  2.2958,   2.204,   2.204,  2.2395,   2.204,  2.1256,  2.1642,  2.1139,   2.204,   2.204,  2.1702,   2.2049,
  //    0.5367, 0.47338, 0.67809, 0.55455, 0.64364, 0.54337,  0.5367, 0.53851, 0.60017, 0.39546, 0.51900, 0.42991, 0.530188,  0.5367, 0.535048,
  //     0.352, 0.29997, 0.46212, 0.37633,  0.4841, 0.41872,   0.352, 0.38744, 0.40562, 0.24347, 0.32926,  0.2214, 0.28687,   0.352, 0.318159;
  // Get the predicted sigma points
  SigmaPointPrediction(&Xsig_pred);
  /*** Compute the predicted mean state and covariance matrix ***/
  // Initialise the predicted mean state vector
  Eigen::VectorXd x(n_x);
  x.fill(0.0);
  // NOTE: if running unit test in `5_tests.cc`, the values of the vector
  // must be set manually using the provided definition
  // x <<
  //    5.93637,
  //    1.49035,
  //    2.20528,
  //   0.536853,
  //   0.353577;
  // Initialise the predicted covariance matrix
  Eigen::MatrixXd P(n_x, n_x);
  P.fill(0.0);
  // NOTE: if running unit test in `5_tests.cc`, the values of the matrix
  // must be set manually using the provided definition
  // P <<
  //   0.0054342,  -0.002405,  0.0034157, -0.0034819, -0.00299378,
  //   -0.002405,    0.01084,   0.001492,  0.0098018,  0.00791091,
  //   0.0034157,   0.001492,  0.0058012, 0.00077863, 0.000792973,
  //  -0.0034819,  0.0098018, 0.00077863,   0.011923,   0.0112491,
  //  -0.0029937,  0.0079109, 0.00079297,   0.011249,   0.0126972;
  PredictMeanAndCovariance(&x, &P);
  /*** Obtain the measurement vector and covariance matrix estimations ***/
  // Instantiate the measurement matrix
  Eigen::MatrixXd Zsig(n_z, n_sigma_points);
  // NOTE: if running unit test in `5_tests.cc`, the values of the matrix
  // must be set manually using the provided definition
  // Zsig <<
  //   6.1190,  6.2334,  6.1531,  6.1283,  6.1143,  6.1190,  6.1221,  6.1190,  6.0079,  6.0883,  6.1125,  6.1248,  6.1190,  6.1188,  6.12057,
  //  0.24428,  0.2337, 0.27316, 0.24616, 0.24846, 0.24428, 0.24530, 0.24428, 0.25700, 0.21692, 0.24433, 0.24193, 0.24428, 0.24515, 0.245239,
  //   2.1104,  2.2188,  2.0639,   2.187,  2.0341,  2.1061,  2.1450,  2.1092,  2.0016,   2.129,  2.0346,  2.1651,  2.1145,  2.0786,  2.11295;
  // Instantiate the mean measurement vector
  Eigen::VectorXd z_pred(n_z);
  // NOTE: if running unit test in `5_tests.cc`, the values of the vector
  // must be set manually using the provided definition
  // z_pred <<
  //   6.12155,
  //    0.245993,
  //    2.10313;
  // Instantiate the predicted measurement covariance matrix
  Eigen::MatrixXd S(n_z, n_z);
  // NOTE: if running unit test in `5_tests.cc`, the values of the matrix
  // must be set manually using the provided definition
  // S <<
  //     0.0946171, -0.000139448,   0.00407016,
  //  -0.000139448,  0.000617548, -0.000770652,
  //    0.00407016, -0.000770652,    0.0180917;
  PredictRadarMeasurement(&z_pred, &S)
  // Instantiate the incoming radar measurement vector
  Eigen::VectorXd z(n_z);
  // NOTE: if running unit test in `5_tests.cc`, the values of the vector
  // must be set manually using the provided definition
  z <<
     5.9214,   // rho in m
     0.2187,   // phi in rad
     2.0062;   // rho_dot in m/s
  /*** Compute the updated state and covariance (innovation step) ***/
  // Initialise the cross-correlation matrix
  Eigen::MatrixXd Tc(n_x, n_z);
  Tc.fill(0.0);
  // Calculate the cross-correlation matrix
  for (int i = 0; i < n_sigma_points; i++) {
    // Compute the difference in the sigma points in state-space
    Eigen::VectorXd x_diff = SigmaPoints::NormaliseHeading(Xsig_pred.col(i) - x); 
    // Compute the difference in the sigma points in measurement-space
    Eigen::VectorXd z_diff = Radar::NormaliseHeading(Zsig.col(i) - z_pred);
    // Compute the cross-correlation for this sigma point
    Tc += w(i) * x_diff * z_diff.transpose(); 
  }
  // Calculate the Kalman gain matrix `K`
  Eigen::MatrixXd K = Tc * S.inverse();
  // Compute the residual
  // i.e., the difference in predicted and received measurement state
  Eigen::VectorXd z_diff = Radar::NormaliseHeading(z - z_pred);
  // Perform the state mean update
  x += K * z_diff;
  // Perform the covariance matrix update
  P -= K * S * K.transpose();
  // Print the resulting outputs
  std::cout << "Updated state x: " << "\n" << x << "\n";
  std::cout << "Updated state covariance P: " << "\n" << P << "\n";
  // Update the input pointers to the outputs
  *x_out = x;
  *P_out = P;
}