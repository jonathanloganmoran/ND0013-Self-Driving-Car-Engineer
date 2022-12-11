/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Main entry file to the code from Lesson 3.1.
 *                       Executes test functions from the `1_tests.cc` file.
 * ----------------------------------------------------------------------------
 */


#include "1_tests.h"


int main() {
  // UNCOMMENT TO RUN EXERCISE 3.1.1 TEST
  // std::cout << "Exercise 3.1.1 : Initialize Priors Function" << "\n";
  // std::cout << "----------------------------------------------" << "\n";
  // test_initialize_priors();
  
  // UNCOMMENT TO RUN EXERCISE 3.1.2 TEST
  // std::cout << "Exercise 3.1.2 : Determine Probabilities" << "\n";
  // std::cout << "----------------------------------------------" << "\n";
  // test_determine_probabilities();

  // UNCOMMENT TO RUN EXERCISE 3.1.3 TEST
  // std::cout << "Exercise 3.1.3 : Motion Model Probability II" << "\n";
  // std::cout << "----------------------------------------------" << "\n";
  // test_motion_model_probability();

  // UNCOMMENT TO RUN EXERCISE 3.1.4 TEST
  // std::cout << "Exercise 3.1.4 : Coding the Motion Model" << "\n";
  // std::cout << "----------------------------------------------" << "\n";
  // test_motion_model();

  // UNCOMMENT TO RUN EXERCISE 3.1.5 TEST
  // std::cout << "Exercise 3.1.5 : Observation Model Probability" << "\n";
  // std::cout << "----------------------------------------------" << "\n";
  // test_observation_model_probability();

  // UNCOMMENT TO RUN EXERCISE 3.1.6 TEST
  // std::cout << "Exercise 3.1.6 : Get Pseudo Ranges" << "\n";
  // std::cout << "----------------------------------------------" << "\n";
  // test_pseudo_range_estimator();

  // UNCOMMENT TO RUN EXERCISE 3.1.7 TEST
  // std::cout << "Exercise 3.1.7 : Coding the Observation Model" << "\n";
  // std::cout << "----------------------------------------------" << "\n";
  // test_observation_model();

  // UNCOMMENT TO RUN EXERCISE 3.1.8 TEST
  std::cout << "Exercise 3.1.8 : Coding the Full Filter" << "\n";
  std::cout << "----------------------------------------------" << "\n";
  test_markov_filter();

  return 0;
}
