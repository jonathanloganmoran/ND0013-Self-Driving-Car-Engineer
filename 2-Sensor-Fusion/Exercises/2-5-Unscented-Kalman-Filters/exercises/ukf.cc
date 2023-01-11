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
 * @param  Xsig_out   Column-vector matrix to store resulting sigma points.
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
  Eigen::MatrixXd Xsig = Eigen::MatrixXd(
      n_x, 
      n_sigma_points
  );
  // Calculate square-root of matrix `P`
  // `A` is the lower-triangular matrix of the Cholesky decomposition
  Eigen::MatrixXd A = P.llt().matrixL();
  /*** Calculate the set of sigma points ***/
  // Set the first column to the mean of the posterior state estimation
  Xsig.col(0) = x;
  // Compute the square-root term for the sigma point vector
  // The square-root of the spreading term
  double spreading_factor = std::sqrt(lambda - n_x);
  // Loop through the columns of `A` to compute columns of `Xsig`
  for (int i = 0; i < n_x + 1; i++) {
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


/**
 * Programming assignment functions: 
 */

void UKF::AugmentedSigmaPoints(MatrixXd* Xsig_out) {

  // set state dimension
  int n_x = 5;

  // set augmented dimension
  int n_aug = 7;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a = 0.2;

  // Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd = 0.2;

  // define spreading parameter
  double lambda = 3 - n_aug;

  // set example state
  VectorXd x = VectorXd(n_x);
  x <<   5.7441,
         1.3800,
         2.2049,
         0.5015,
         0.3528;

  // create example covariance matrix
  MatrixXd P = MatrixXd(n_x, n_x);
  P <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;

  // create augmented mean vector
  VectorXd x_aug = VectorXd(7);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  /**
   * Student part begin
   */
 
  // create augmented mean state

  // create augmented covariance matrix

  // create square root matrix

  // create augmented sigma points
  
  /**
   * Student part end
   */

  // print result
  std::cout << "Xsig_aug = " << std::endl << Xsig_aug << std::endl;

  // write result
  *Xsig_out = Xsig_aug;
}

/** 
 * expected result:
 *  Xsig_aug =
 * 5.7441  5.85768   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441  5.63052   5.7441   5.7441   5.7441   5.7441   5.7441   5.7441
 *   1.38  1.34566  1.52806     1.38     1.38     1.38     1.38     1.38  1.41434  1.23194     1.38     1.38     1.38     1.38     1.38
 * 2.2049  2.28414  2.24557  2.29582   2.2049   2.2049   2.2049   2.2049  2.12566  2.16423  2.11398   2.2049   2.2049   2.2049   2.2049
 * 0.5015  0.44339 0.631886 0.516923 0.595227   0.5015   0.5015   0.5015  0.55961 0.371114 0.486077 0.407773   0.5015   0.5015   0.5015
 * 0.3528 0.299973 0.462123 0.376339  0.48417 0.418721   0.3528   0.3528 0.405627 0.243477 0.329261  0.22143 0.286879   0.3528   0.3528
 *      0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641        0
 *      0        0        0        0        0        0        0  0.34641        0        0        0        0        0        0 -0.34641
 */