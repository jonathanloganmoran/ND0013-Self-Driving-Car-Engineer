/* ------------------------------------------------------------------------------
 * Lesson "5.2: Vehicle Motion Models"
 * Authors     : David Siller, Andrew Gray, Dominique Luna.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com) 
 *
 * Purpose of this file: Header file for the Global Kinematic motion model.
 * ----------------------------------------------------------------------------
 */

#include "1_global_kinematic_model.h"



/* Evalautes the global kinematic motion model for the given values.
 * 
 * The global kinematic motion model is a simplified expression of vehicle
 * kinematics with state values defined in the global reference frame.
 * The inputs to the model are the state vector containing the vehicle position,
 * the vehicle heading and the vehicle velocity at the time-step $t$. The
 * next-state is calculated by evaluating the global kinematic model which
 * neglects the effects of gravity and any internal / external vehicle forces.
 * 
 * To evaluate the result of the global kinematic model function, we initialise
 * the starting state vector and next-state input actuations. After performing
 * the state vector update, we compare its values to the expected. If the
 * resulting values are within a margin of error given by `epsilon`, we
 * conclude that the function performs as expected.
 */
void test_global_kinematic_model() {
  // [x, y, psi, v]
  Eigen::VectorXd state(4);
  // [delta, v]
  Eigen::VectorXd actuators(2);
  state << 0, 0, deg2rad(45), 1;
  actuators << deg2rad(5), 1;
  // should be [0.212132, 0.212132, 0.798488, 1.3]
  Eigen::VectorXd next_state = global_kinematic(
      state, 
      actuators, 
      0.3
  );
  Eigen::VectorXd next_state_expected(4, 1);
  next_state_expected <<
    0.212132, 
    0.212132, 
    0.798488, 
    1.3;
  // Amount of L2 distance error permitted between actual and expected
  double epsilon = 0.001;
  std::cout << next_state << "\n";
  std::cout << "`next_state` matches `next_state_expected` ";
  std::cout << "(with `epsilon = " << epsilon << " ): ";
  std::cout << std::boolalpha << next_state.isApprox(next_state_expected);
  std::cout << "\n";
}


int main() {
  // Extracurricular Exercise 5.2.1: Global Kinematic Model
  // test_global_kinematic_model();
  return 0;
}