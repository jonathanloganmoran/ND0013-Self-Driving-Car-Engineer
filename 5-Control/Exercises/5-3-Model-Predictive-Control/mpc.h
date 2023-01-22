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

//#include "helpers.h"                            // `polyfit()`, `polyeval()` 
#include "eigen-3.3.7/Eigen/Core"               // `Eigen::VectorXd`
#include <cppad/cppad.hpp>                      // `CppAD::AD`
#include <cppad/ipopt/solve.hpp>                // `CppAD::ipopt::solve`
#include <math.h>
#include <cmath>
#include <vector>


/* The Model Predictive Control (MPC) controller class.
 *
 * The MPC consists of solving a non-linear optimisation problem with the
 * IPOPT algorithm. 
 * 
 * The MPC controller here is used to find a cost-minimal trajectory of
 * next-state waypoints realised over a pre-defined time horizon of `N` steps.
 * Each next-state waypoint is iteratively computed and selected as the
 * next-best lowest-cost state of the vehicle for each simulated trajectory.
 * This is done by solving the non-linear function formed by the polynomial.
 * For each trajectory simulated across a time-horizon of `N` time-steps, a
 * set of actuator commands are realised ("performed"); their associated cost
 * is computed and minimised w.r.t. the following three cost functions:
 *    1. Deviation from reference trajectory;
 *    2. Comfort constraints;
 *    3. Efficiency constraints. 
 * These three cost functinos are used to penalise the vehicle behaviour in
 * each simulated trajectory and ultimately guide the optimisation problem
 * towards a cost-minimal solution, i.e., the most-desirable next-state for
 * the vehicle is selected based on a realised time-horizon. Once the
 * cost-minimal next-state has been selected, the remaining `N-1` waypoints
 * of the trajectory are discarded. A new trajectory estimate is then formed
 * and the cost-minimisation problem is repeated.
 */
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