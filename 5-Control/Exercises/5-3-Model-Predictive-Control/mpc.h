/* ----------------------------------------------------------------------------
 * Lesson "5.3: Model Predictive Control"
 * Authors     : David Siller, Andrew Gray, Dominique Luna.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com) 
 *
 * Purpose of this file: Header file for Model Predictive Control functions.
 * ----------------------------------------------------------------------------
 */

#ifndef MPC_H
#define MPC_H

#include "Eigen-3.3/Eigen/Core"
#include <vector>


class MPC {
 public:
  MPC();
  virtual ~MPC();
  // Returns the next-state of the model for a given initial state
  std::vector<double> solve_controller(
      const Eigen::VectorXd& x0, 
      const Eigen::VectorXd& coeffs
  );
};

#endif  // MPC_H