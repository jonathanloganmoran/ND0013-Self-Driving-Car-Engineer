/* ----------------------------------------------------------------------------
 * Lesson "5.3: Model Predictive Control"
 * Authors     : David Siller, Andrew Gray, Dominique Luna.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com) 
 *
 * Purpose of this file: Performs lane-keeping with Model Predictive Control.
 * ----------------------------------------------------------------------------
 */

#include "mpc.h"
#include "helpers.h"                 // Get coefficients of the polynomial
#include "matplotlibcpp.h"           // Plot controller and actuator values
#include <vector>                    // Store simulation history
#include <cstring>
#include <string>                    // Matplotlib plotting parameters
#include <map>                       // Matplotlib plotting parameters

namespace plt = matplotlibcpp;


/* Runs the vehicle simulation.
 *
 * The vehicle "Follow Lane" manoeuvre is performed using the Model Predictive 
 * Control (MPC)-based controller. The MPC consists of solving a non-linear
 * optimisation problem in order to compute the next-state of the vehicle over
 * a pre-defined time horizon of `N` steps. The desired lane-keeping manoeuvre
 * is performed by iteratively computing the next-best lowest-cost state of the
 * vehicle. This is done by solving the non-linear function formed by the
 * polynomial using the IPOPT algorithm. The MPC iterates over the trajectory
 * waypoints and simulates the vehicle state over `N` number of time-steps.
 * A set of actuator commands are realised ("performed") for each simulation
 * step and their associated cost is computed and minimised w.r.t. the three
 * cost functions defined in `MPC`:
 *    1. Deviation from reference trajectory;
 *    2. Comfort constraints;
 *    3. Efficiency constraints. 
 * These three cost functions are used to penalise the vehicle behaviour and
 * guide the optimisation problem towards a cost-minimal solution, i.e., the
 * most-desirable next-state for the vehicle based on a realised time-horizon.
 * Once the cost-minimal next-state of the vehicle has been selected, the
 * remaining waypoints of the trajectory are discarded; a new trajectory
 * estimate is then formed and the cost-minimisation problem is repeated.
 */
int main() {
  MPC mpc;
  // Set the number of simulation iterations to perform
  // CANDO: modify this value
  int iters = 50;
  Eigen::VectorXd ptsx(2);
  Eigen::VectorXd ptsy(2);
  ptsx << -100, 100;
  ptsy << -1, -1;
  // Fitting a first-order polynomial, i.e., the reference line
  auto coeffs = polyfit(ptsx, ptsy, 1);
  // Set the initial state values
  // CANDO: modify these values
  double x = -1;
  double y = 10;
  double psi = 0;
  double v = 10;
  // Compute the cross-track error (CTE), i.e., the difference
  // between current position and reference line about the $y$-axis 
  double cte = polyeval(coeffs, x) - y;
  // Compute the orientation error, i.e., the difference
  // between the desired heading and the angle tangential to reference line 
  double epsi = psi - std::atan(coeffs[1]);
  Eigen::VectorXd state(6);
  // Read in the initial state vector values
  state << x, y, psi, v, cte, epsi;
  // Add the initial state values to the state history vectors
  std::vector<double> x_vals = {state[0]};
  std::vector<double> y_vals = {state[1]};
  std::vector<double> psi_vals = {state[2]};
  std::vector<double> v_vals = {state[3]};
  std::vector<double> cte_vals = {state[4]};
  std::vector<double> epsi_vals = {state[5]};
  // Add the initial actuator values to the actuator history vectors
  std::vector<double> delta_vals = {};
  std::vector<double> a_vals = {};
  // Perform the simulation iterations
  for (size_t i = 0; i < iters; ++i) {
    std::cout << "Iteration " << i << "\n";
    // Solve the non-linear optimisation problem with IPOPT
    auto vars = mpc.solve_controller(state, coeffs);
    // Append the next-state values to the state history vectors
    x_vals.push_back(vars[0]);
    y_vals.push_back(vars[1]);
    psi_vals.push_back(vars[2]);
    v_vals.push_back(vars[3]);
    cte_vals.push_back(vars[4]);
    epsi_vals.push_back(vars[5]);
    // Append the next-state actuator values to the actuator history vectors
    delta_vals.push_back(vars[6]);
    a_vals.push_back(vars[7]);
    // Update the current state with the next-state values
    state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
    std::cout << "x = " << vars[0] << "\n";
    std::cout << "y = " << vars[1] << "\n";
    std::cout << "psi = " << vars[2] << "\n";
    std::cout << "v = " << vars[3] << "\n";
    std::cout << "cte = " << vars[4] << "\n";
    std::cout << "epsi = " << vars[5] << "\n";
    std::cout << "delta = " << vars[6] << "\n";
    std::cout << "a = " << vars[7] << "\n";
    std::cout << "\n";
  }
  /*** Plotting the actuator and error values with Matplotlib ***/
  // CANDO: modify the plotting for debugging or stylistic purposes
  plt::subplot(3, 1, 1);
  std::map<std::string, std::string> mpl_params;
  mpl_params["fontname"] = "Times New Roman";
  mpl_params["fontsize"] = "24";
  plt::suptitle(
      "Model Predictive Control (MPC) for Vehicle 'Follow Lane' Trajectory",
      mpl_params
  );
  mpl_params["fontsize"] = "20";
  plt::title("Cross-Track Error — CTE (m)",
      mpl_params
  );
  plt::plot(cte_vals, "r"
      // {{"fmt", "r"}, {"label", "\mathrm{CTE}"}}
  );
  plt::subplot(3, 1, 2);
  plt::title("Steering Input — Delta (radians)",
      mpl_params
  );
  plt::plot(delta_vals, "g"
      // {{"fmt", "g"}, {"label", "$\delta$ (rad)"}}
  );
  plt::subplot(3, 1, 3);
  plt::title("Velocity (m/s)",
      mpl_params
  );
  plt::plot(v_vals, "b"
      // {{"fmt" , "b"}, {"label", "$\mathrm{m}/\mathrm{s}"}}
  );
  //plt::legend();
  plt::show();
}