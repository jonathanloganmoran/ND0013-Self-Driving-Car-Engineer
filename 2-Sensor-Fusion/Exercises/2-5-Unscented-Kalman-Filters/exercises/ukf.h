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


/* The Unscented Kalman Filter (UKF) class.
 */
class UKF {
 public:
  UKF();
  virtual ~UKF();
  // Initalises the UKF instance
  void InitFilter();
  // Computes the sigma points
  void GenerateSigmaPoints(
      Eigen::MatrixXd* Xsig_out
  );
  void AugmentedSigmaPoints(
      Eigen::MatrixXd* Xsig_out
  );
  void SigmaPointPrediction(
      Eigen::MatrixXd* Xsig_out
  );
  void PredictMeanAndCovariance(
      Eigen::VectorXd* x_pred, 
      Eigen::MatrixXd* P_pred
  );
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