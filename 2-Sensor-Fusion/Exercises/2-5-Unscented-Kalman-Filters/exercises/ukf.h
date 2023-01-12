/* ----------------------------------------------------------------------------
 * Lesson "2.5: Unscented Kalman Filters"
 * Authors     : Dominik Nuss, Andrei Vatavu.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the Unscented Kalman Filter (UKF).
 * ----------------------------------------------------------------------------
 */

#ifndef UKF_H
#define UKF_H

#include "Dense"
#include <random>
#include <cmath>


/* The Unscented Kalman Filter (UKF) class.
 */
class UKF {
 public:
  UKF();
  virtual ~UKF();
  // Initalises the UKF instance
  void InitFilter();
  // Computes the sigma points from the state vector / covariance matrix
  void GenerateSigmaPoints(
      Eigen::MatrixXd* Xsig_out
  );
  // Computes the sigma points from the augemnted state / covariance
  void AugmentedSigmaPoints(
      Eigen::MatrixXd* Xsig_out
  );
  // Evaluates the state prediction / state transition function 
  void SigmaPointPrediction(
      Eigen::MatrixXd* Xsig_out
  );
  // Normalises heading angle of state estimation to a value in range [-pi, pi]
  Eigen::VectorXd NormaliseHeading(
      Eigen::VectorXd Xsig_pred_diff
  );
  // Performs the prediction of the state estimation and covariance matrix
  void PredictMeanAndCovariance(
      Eigen::VectorXd* x_pred, 
      Eigen::MatrixXd* P_pred
  );
  // Predicts the radar measurement vector and covariance in innovation step
  void PredictRadarMeasurement(
      Eigen::VectorXd* z_out, 
      Eigen::MatrixXd* S_out
  );
  void UpdateState(
      Eigen::VectorXd* x_out, 
      Eigen::MatrixXd* P_out
  );
};


#endif  // UKF_H