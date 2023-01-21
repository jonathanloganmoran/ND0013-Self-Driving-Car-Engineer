/* ----------------------------------------------------------------------------
 * Lesson "5.3: Model Predictive Control"
 * Authors     : David Siller, Andrew Gray, Dominique Luna.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com) 
 *
 * Purpose of this file: Implements controller with Model Predictive Control.
 * ----------------------------------------------------------------------------
 */

#include "mpc.h"
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <vector>
#include "Eigen-3.3/Eigen/Core"

//using CppAD::AD;
//using Eigen::VectorXd;
/*** Initialising the Model Predictive Control (MPC) hyperparameters ****/
// NOTE: All programme and MPC constants are defined with a leading 'k'
/**
 * TODO: Set N and dt
 */
// Number of time-steps of the time horizon
const size_t kN = ? ;
// Delta-time, i.e., elapsed time (s) between actuations
const double kDt = ? ;

// This is the length from front to CoG that has a similar radius.
// Length (m) taken from the vehicle's centre of mass (CoG) to the front axle
// NOTE: this value has been estimated via simulation, i.e., obtained by
// measuring the radius formed by moving vehicle in circle with constant
// steering angle and constant velocity on flat terrian. The `kL_f` parameter
// here was tuned until the simulation radius was congruent with circle.
const double kL_f = 2.67;

// Vehicle reference velocity (m/s)
// CANDO: modify this value.
double kRef_v = 40;

/*** Defining state / actuator vector boundaries ****/
// The solver assumes the state variables and actuator variables are stored
// in a singular vector. Thus, we define here the boundaries of the variables'
// values in the vector as given by the indices here.
const size_t kX_start = 0;
const size_t kY_start = kX_start + kN;
const size_t kPsi_start = kY_start + kN;
const size_t kV_start = kPsi_start + kN;
const size_t kCte_start = kV_start + kN;
const size_t kEpsi_start = kCte_start + kN;
const size_t kDelta_start = kEpsi_start + kN;
const size_t kA_start = kDelta_start + kN - 1;


/* Solver for the non-linear model used in the MPC controller. 
 *
 * Implements the `CppAD::AD::solver()` automatic differentiation function.
 * 
 * @var  coeffs   Coefficients of the polynomial function to solve.
 */
class FG_eval {
 public:
  Eigen::VectorXd coeffs;
  // Initialise the class instance with the given polynomial coefficients
  FG_eval(Eigen::VectorXd coeffs) : this->coeffs(coeffs) {}
  //typedef CPPAD_TESTVECTOR(CppAD::AD::AD<double>) ADvector;
  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(
      CppAD::AD<double>& fg, 
      const CppAD::AD<double>& vars
  ) {
    // The cost is assumed to be stored as the first element of `fg`.
    // Any additions to the cost value should be added to `fg[0]`.
    fg[0] = 0;
    // Reference state cost value
    /**
     * TODO: Define the cost related the reference state and
     *   anything you think may be beneficial.
     */
    /*** Defining the model constraints ***/
    // Set the initial model constraints
    // We add `1` to each of the starting indices since
    // cost is located at index 0 of `fg`.
    fg[1 + kX_start] = vars[kX_start];
    fg[1 + kY_start] = vars[kY_start];
    fg[1 + kPsi_start] = vars[kPsi_start];
    fg[1 + kV_start] = vars[kV_start];
    fg[1 + kCte_start] = vars[kCte_start];
    fg[1 + kEpsi_start] = vars[kEpsi_start];
    // Setting the remaining model constraints
    for (int t = 1; t < kN; ++t) {
      /**
       * TODO: Grab the rest of the states at t+1 and t.
       *   We have given you parts of these states below.
       */
      CppAD::AD<double> x1 = vars[kX_start + t];
      CppAD::AD<double> x0 = vars[kX_start + t - 1];
      CppAD::AD<double> psi0 = vars[kPsi_start + t - 1];
      CppAD::AD<double> v0 = vars[kV_start + t - 1];
      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      // NOTE: The use of `AD<double>` and use of `CppAD`!
      // CppAD can compute derivatives and pass these to the solver
      /**
       * TODO: Setup the rest of the model constraints
       */
      fg[1 + kX_start + t] = (
          x1 - (x0 + v0 * CppAD::cos(psi0) * kDt)
      );
    }
  }
};


// MPC class definition
MPC::MPC() {}
MPC::~MPC() {}


/* Implements the MPC non-linear optimisation function solver. 
 *
 * The MPC controller defined here is used to perform a "follow" manoeuvre
 * with minimal cost. A "follow" manoeuvre consists of maintaining a trajectory
 * parallel with and as close to the lane centre-line as possible.
 * The costs minimised here are the cross-track error (CTE), i.e., the
 * displacement from the lane centre-line along the $y$-axis, and the
 * psi-angle error (ePSI), i.e., the difference between the current and the
 * desired vehicle heading.
 * 
 * Used here to solve the non-linear function is the `CppAD::AD::solver()`
 * function which uses automatic differentiation (AD) to estimate the
 * Jacobian and Hessians needed to derive the cost-minimised vector of 
 * next-state values.  
 * 
 * @param    x0       Vector of initial state values.
 * @param    coeffs   Coefficients of the polynomial function.
 * @returns  Vector of cost-minimised state values (i.e., the solution).
 */
std::vector<double> MPC::solve_controller(
    const Eigen::VectorXd& x0, 
    const Eigen::VectorXd& coeffs
) {
  //typedef CPPAD_TESTVECTOR(double) Dvector;
  double x = x0[0];
  double y = x0[1];
  double psi = x0[2];
  double v = x0[3];
  double cte = x0[4];
  double epsi = x0[5];
  // Set the number of independent variables
  // N timesteps == N - 1 actuations
  size_t n_vars = kN * 6 + (kN - 1) * 2;
  // Number of constraints
  size_t n_constraints = kN * 6;
  // Initial value of the independent variables
  // Should be 0 except for the initial values.
  //CppAD::AD::Dvector vars(n_vars);
  CppAD::AD<double> vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  // Lower and upper limits for x
  CppAD::AD<double> vars_lowerbound(n_vars);
  CppAD::AD<double> vars_upperbound(n_vars);
  // Set all non-actuators upper and lower limits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians)
  // CANDO: Modify these values.
  for (int i = delta_start; i < a_start; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  // Acceleration / decceleration upper and lower limits
  // CANDO: Modify these values.
  for (int i = a_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  CppAD::AD<double> constraints_lowerbound(n_constraints);
  CppAD::AD<double> constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; ++i) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[kX_start] = x;
  constraints_lowerbound[kY_start] = y;
  constraints_lowerbound[kPsi_start] = psi;
  constraints_lowerbound[kV_start] = v;
  constraints_lowerbound[kCte_start] = cte;
  constraints_lowerbound[kEpsi_start] = epsi;
  constraints_upperbound[kX_start] = x;
  constraints_upperbound[kY_start] = y;
  constraints_upperbound[kPsi_start] = psi;
  constraints_upperbound[kV_start] = v;
  constraints_upperbound[kCte_start] = cte;
  constraints_upperbound[kEpsi_start] = epsi;
  // Initialise the class that computes objective and constraints
  FG_eval fg_eval(coeffs);
  // Setting the differentiation options
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // Store the returned solution
  CppAD::ipopt::solve_result<CppAD::AD<double>> solution;
  // Solve the non-linear optimisation problem using automatic differentiation
  CppAD::ipopt::solve<CppAD::AD<double>, FG_eval>(
      options,
      vars, 
      vars_lowerbound, 
      vars_upperbound, 
      constraints_lowerbound,
      constraints_upperbound, 
      fg_eval, 
      solution
  );
  // Perform sanity check on some of the solution values
  bool ok = true;
  ok &= (solution.status 
         == CppAD::ipopt::solve_result<CppAD::AD<double>>::success
  );
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;
  return {
      solution.x[kX_start + 1],
      solution.x[kY_start + 1],
      solution.x[kPsi_start + 1], 
      solution.x[kV_start + 1],
      solution.x[kCte_start + 1], 
      solution.x[kEpsi_start + 1],
      solution.x[kDelta_start], 
      solution.x[kA_start]
  };
}