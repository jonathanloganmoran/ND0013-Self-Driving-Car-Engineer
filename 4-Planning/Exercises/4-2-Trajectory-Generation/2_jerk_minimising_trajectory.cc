/* ------------------------------------------------------------------------------
 * Lesson "4.2: Trajectory Generation"
 * Authors     : Sebastian Thrun, Emmanuel Boidot of Mercedes-Benz R&D.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the quintic polynomial solver. This solver
 *                       is used to generate jerk-minimised trajectories from
 *                       the `start` to `end` states described by the 1D
 *                       kinematics along the lateral and longitudinal axes. 
 * ----------------------------------------------------------------------------
 */

#include "2_jerk_minimising_trajectory.h"


/* Returns the coefficients of the quintic polynomial.
 * 
 * The quintic polynomial is used to estimate the minimum-jerk trajectory in 1D
 * along the lateral and longitudinal axes of the Frenet coordinate frame. The
 * trajectory is computed from the `start` to `end` states for an elapsed time
 * `T`.
 * 
 * Here each state is a vector of 1D kinematics values describing the position,
 * velocity and acceleration along either the lateral $d$ or longitudinal $s$
 * axis. For example, the `start` state for the lateral motion along $s$ will
 * be the vector of values described by:
 *    $[s_{i}, \dot{s}_{i}, \ddot{s}_{i}]$,
 * which are the first-, second- and third-order derivatives of the lateral
 * position $s(t)$.
 * 
 * Note that the quintic polynomial equations for position, velocity, and
 * acceleration are evaluated at $t = 0$ to obtain the first three coefficients
 * $[\alpha_{0}, \alpha_{1}, \alpha_{2}] = [s_{i}, \dot{s}_{i}, \ddot{s}_{i}/2].
 * The last three coefficients are computed with the quintic polynomial solver.
 * 
 * @param    start    1D kinematics of the starting vehicle state. 
 * @param    end      1D kinematics of the final vehicle state.
 * @param    T        Elapsed time (s) over which the manoeuvre should occur.
 * @returns  coeffs   Vector of six coefficients for the quintic polynomial.
 */
std::vector<double> JMT(
    std::vector<double>& start, 
    std::vector<double>& end, 
    double T
) {
  std::vector<double> coeffs;
  // Setting the first three known coefficients given by `start` when $t = 0$
  coeffs = {start[0], start[1], 0.5 * start[2]};
  // Creating the matrix equation
  Eigen::MatrixXd A(3, 3);
  A << 
      std::pow(T,3), std::pow(T,4), std::pow(T,5),
      3 * std::pow(T,2), 4 * std::pow(T,3), 5 * std::pow(T,4),
      6 * std::pow(T,1), 12 * std::pow(T,2), 20 * std::pow(T,3);
  Eigen::VectorXd b(3, 1);
  b <<
      end[0] - (start[0] + start[1] * T + 0.5 * start[2] * T * T),
      end[1] - (start[1] + start[2] * T),
      end[2] - start[2];
  // Using the linear system of equations form to compute the last three coeffs
  Eigen::VectorXd x(3, 1);
  x << A.inverse() * b;
  // Copying the last three computed coefficients to the output vector
  coeffs.push_back(x[0]);
  coeffs.push_back(x[1]);
  coeffs.push_back(x[2]); 
  return coeffs;
}