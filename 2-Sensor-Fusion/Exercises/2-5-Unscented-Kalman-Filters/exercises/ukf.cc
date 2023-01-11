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