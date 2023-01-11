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
  Eigen::VectorXd x = Eigen::VectorXd(n_x);
  x << 5.7441,
       1.3800,
       2.2049,
       0.5015,
       0.3528;
  // Set the covariance matrix values
  // Here, this is assumed to be the covariance of posterior state estimation
  Eigen::MatrixXd P = Eigen::MatrixXd(n_x, n_x);
  P << 0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
      -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
       0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
      -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
      -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
  // Create the sigma point matrix
  Eigen::MatrixXd Xsig = Eigen::MatrixXd(n_x, n_sigma_points);
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
    MatrixXd* Xsig_out
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
  VectorXd nu_k(n_a, 1);
  nu_k << nu_a(rand_gen),
          nu_yawdd(rand_gen);
  // Set the state vector values
  // Here, this is assumed to be the mean of the posterior state estimation
  Eigen::VectorXd x = Eigen::VectorXd(n_x);
  x << 5.7441,
       1.3800,
       2.2049,
       0.5015,
       0.3528;
  // Set the covariance matrix values
  // Here, this is assumed to be the covariance of posterior state estimation
  Eigen::MatrixXd P = Eigen::MatrixXd(n_x, n_x);
  P << 0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
      -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
       0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
      -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
      -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
  // Instantiate the augmented mean vector
  Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug);
  // Instantiate the augmented state covariance matrix
  Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug, n_aug);
  // Instantiate the augmented sigma point matrix
  Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug, n_sigma_points);
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
    MatrixXd* Xsig_out
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
  MatrixXd Xsig_pred = MatrixXd(n_x, n_sigma_points);
  // Define the delta-time variable (s)
  double delta_t = 0.1;
  /**
   * Student part begin
   */
  // Predict the sigma points by evaulting the process model function
  // Avoid a divide-by-zero error
  // Write the predicted sigma points into right-column of the output matrix
  /**
   * Student part end
   */
  // Print the resulting predicted sigma point matrix
  std::cout << "Xsig_pred = " << "\n" << Xsig_pred << "\n";
  // Write the result to the output matrix
  *Xsig_out = Xsig_pred;
}