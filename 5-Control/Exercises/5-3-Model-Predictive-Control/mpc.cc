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

//typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
//typedef CPPAD_TESTVECTOR(double) Dvector;

//using CppAD::AD;
//using Eigen::VectorXd;
/*** Initialising the Model Predictive Control (MPC) hyperparameters ****/
// NOTE: All programme and MPC constants are defined with a leading 'k'
/**
 * TODO: Set N and dt
 */
// Number of time-steps of the time horizon
const size_t kN = 9;
// Delta-time, i.e., elapsed time (s) between actuations
const double kDt = 0.5;
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


// MPC class definition
MPC::MPC() {}
MPC::~MPC() {}


/* Solver for the non-linear model used in the MPC controller. 
 *
 * Implements the `CppAD::AD::solver()` automatic differentiation function.
 * 
 * @var  coeffs   Coefficients of the polynomial function to solve.
 */
class FG_eval {
 public:
  // Used in IPOPT solver (`cppad/ipopt/solve.hpp`)
  typedef CPPAD_TESTVECTOR(CppAD::AD<double>) ADvector;
  // Stores the coefficients of the polynomial function
  Eigen::VectorXd coeffs;
  // Initialise the class instance with the given polynomial coefficients
  FG_eval(Eigen::VectorXd coeffs) : coeffs(coeffs) {}
  //typedef CPPAD_TESTVECTOR(CppAD::AD::AD<double>) ADvector;

  /* Updates the state and actuator values with the computed values. 
   *
   * @param  fg    Vector containing the cost and constraints.
   * @param  vars  Vector containing the variable values (state & actuators).
   */
  void operator() (
      ADvector& fg, 
      const ADvector& vars
  ) {
    /*** Computing the cost functions ***/
    // NOTE: the cost is assumed to be stored as the first element of `fg`,
    // any additions to the cost value should be added to `fg[0]`.
    fg[0] = 0;
    // Compute the costs associated with deviation from the reference state
    for (int t = 0; t < kN; ++t) {
      // Penalising deviation from reference velocity
      fg[0] += CppAD::pow(
          (vars[kV_start + t] - kRef_v), 2
      );
      // Penalising deviation from reference trajectory (i.e, the CTE)
      fg[0] += CppAD::pow(vars[kCte_start + t], 2);
      // Penalising deviation from heading angle (i.e., ePSI)
      fg[0] += CppAD::pow(vars[kEpsi_start + t], 2);
    } 
    // Compute the costs associated with comfort constraints
    for (int t = 0; t < kN - 2; ++t) {
      // Penalising higher change in acceleration values
      fg[0] += CppAD::pow(
          (vars[kA_start + t + 1] - vars[kA_start + t]), 2
      );
      // Penalising higher change in steering input
      fg[0] += CppAD::pow(
          (vars[kDelta_start + t + 1] - vars[kDelta_start + t]), 2
      );
    }
    // Compute the costs associated with efficiency constraints
    // NOTE: we use these costs to encourage goal-directed behaviour
    // of the vehicle in a lane-keeping manoevure
    // CANDO: modify these costs to accomodate the desired scenario
    // i.e., decrease cost associated with steering for urban driving
    for (int t = 0; t < kN - 1; ++t) {
      // Penalising use of throttle (acceleration), since we want to
      // encourage a fuel-efficient behaviour in this nominal scenario
      fg[0] += CppAD::pow(vars[kA_start + t], 2);
      // Penalising use of steering input, since we want the vehicle to
      // not veer too significantly from the in-lane reference trajectory
      fg[0] += CppAD::pow(vars[kDelta_start + t], 2);
    }
    /*** Updating the model constraints (state and actuator commands) ***/
    // Set the initial model constraints
    // Adding `1` to starting indices since cost is the first element `fg[0]`
    fg[kX_start + 1] = vars[kX_start];
    fg[kY_start + 1] = vars[kY_start];
    fg[kPsi_start + 1] = vars[kPsi_start];
    fg[kV_start + 1] = vars[kV_start];
    fg[kCte_start + 1] = vars[kCte_start];
    fg[kEpsi_start + 1] = vars[kEpsi_start];
    // Setting the remaining model constraints
    for (int t = 1; t < kN; ++t) {
      // Get the previous and current state values
      CppAD::AD<double> x0 = vars[kX_start + t - 1];
      CppAD::AD<double> x1 = vars[kX_start + t];
      CppAD::AD<double> y0 = vars[kY_start + t - 1];
      CppAD::AD<double> y1 = vars[kY_start + t];
      CppAD::AD<double> psi0 = vars[kPsi_start + t - 1];
      CppAD::AD<double> psi1 = vars[kPsi_start + t];
      CppAD::AD<double> v0 = vars[kV_start + t - 1];
      CppAD::AD<double> v1 = vars[kV_start + t];
      CppAD::AD<double> cte0 = vars[kCte_start + t - 1];
      CppAD::AD<double> cte1 = vars[kCte_start + t];
      CppAD::AD<double> epsi0 = vars[kEpsi_start + t - 1];
      CppAD::AD<double> epsi1 = vars[kEpsi_start + t];
      // Get the previous actuator values
      // NOTE: accessing "current" values throws out-of-bounds error
      // i.e., current actuator values not stored in the `vars` vector
      CppAD::AD<double> delta0 = vars[kDelta_start + t - 1];
      //CppAD::AD<double> delta1 = vars[kDelta_start + t];
      CppAD::AD<double> a0 = vars[kA_start + t - 1];
      //CppAD::AD<double> a1 = vars[kA_start + t];
      // Compute the line tangential to the first-order polynomial,
      // i.e., the reference line $\mathcal{f} = a0 + a1 * x_1$
      CppAD::AD<double> f_x0 = this->coeffs[0] + this->coeffs[1] * x0;
      // Evaluating the first-order derivative of $\mathrm{f}$ at $x_t$
      CppAD::AD<double> psi0_des = CppAD::atan(coeffs[1]);
      // Updating all state values to the next-state
      fg[kX_start + t + 1] = (
          x1 - (x0 + v0 * CppAD::cos(psi0) * kDt)
      );
      fg[kY_start + t + 1] = (
          y1 - (y0 + v0 * CppAD::sin(psi0) * kDt)
      );
      fg[kPsi_start + t + 1] = (
          psi1 - (psi0 + (v0 / kL_f) * delta0 * kDt)
      );
      fg[kV_start + t + 1] = (
          v1 - (v0 + a0 * kDt)
      );
      fg[kCte_start + t + 1] = (
          cte1 - ((f_x0 - y0) + (v0 * CppAD::sin(epsi0) * kDt))
      );
      fg[kEpsi_start + t + 1] = (
          epsi1 - (psi0 - psi0_des + (v0 / kL_f) * delta0 * kDt)
      );
    }
  }
};


/* Implements the IPOPT non-linear optimisation function solver for the MPC. 
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
 * IPOPT function which uses automatic differentiation (AD) to estimate the
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
  // Used in IPOPT solver 
  typedef CPPAD_TESTVECTOR(double) Dvector;
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
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; ++i) {
    vars[i] = 0.0;
  }
  // Set the initial variable values
  vars[kX_start] = x;
  vars[kY_start] = y;
  vars[kPsi_start] = psi;
  vars[kV_start] = v;
  vars[kCte_start] = cte;
  vars[kEpsi_start] = epsi;
  // Lower and upper limits for x
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators upper and lower limits
  // to the max negative and positive values.
  for (int i = 0; i < kDelta_start; ++i) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians)
  // CANDO: Modify these values.
  for (int i = kDelta_start; i < kA_start; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  // Acceleration / decceleration upper and lower limits
  // CANDO: Modify these values.
  for (int i = kA_start; i < n_vars; ++i) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }
  // Lower and upper limits for constraints
  // All of these should be 0 except the initial
  // state indices.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
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
  CppAD::ipopt::solve_result<Dvector> solution;
  // Solve the non-linear optimisation problem using automatic differentiation
  CppAD::ipopt::solve<Dvector, FG_eval>(
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
         == CppAD::ipopt::solve_result<Dvector>::success
  );
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << "\n";
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