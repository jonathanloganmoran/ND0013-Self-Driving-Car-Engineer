/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Tests the functions from Lesson 3.1.
 * ----------------------------------------------------------------------------
 */


/* Tests the `initialize_priors` function inside `1_initialize_priors.cc`.
 * Returns the initialised 1-D prior probability vector.
 */
void test_initialize_priors() {
  // Set standard deviation of position
  float position_stdev = 1.0f;
  // Set map horizon distance in meters
  int map_size = 25;
  // Initialise landmarks
  std::vector<float> landmark_positions{5, 10, 20};
  // Testing initialise priors function
  std::vector<float> priors = initialize_priors(map_size,
                                                landmark_positions,
                                                position_stdev
  );
  // Print the probability values to `stdout`
  for (int p = 0; p < priors.size(); ++p) {
    std::cout << priors[p] << "\n";
  }
}


/* Tests the `normpdf` function inside `2_determine_probabilities.cc`.
 * Returns the probability vector from the normal distribution
 * parameterised with the given scalar values of (`mu`, `sigma`).
 */
void test_determine_probabilities() {
  // The position `x`
  float value = 1.0;
  // The position / observation control parameter
  float parameter = 1.0;    // Number of units moved each time-step
  // The position / observation standard deviation
  float stdev = 1.0;
  // Compute the probability distribution for the given values
  static float prob = Helpers::normpdf(value, parameter, stdev);
  // Print the returned probability value
  std::cout << prob << std::endl;
}


/* Tests the `normpdf` function inside `2_determine_probabilities`.
 * Returns the probability value obtained from the normal distribution
 * parameterised with the given scalar values of (`mu`, `sigma`).
 */
void test_motion_model_probability() {
  // Defining the distribution parameters
  float control_parameter = 1.0f;    // Number of units moved each time-step
  float position_stdev = 1.0f;       // Position / observation standard deviation
  // Computing the position delta
  int x_pseudo = 7;
  int x_pre_pseudo = 5;
  float x_delta = x_pseudo - x_pre_pseudo;
  std::cout << "Position delta: " << x_delta << "\n";
  // Computing the probability value
  static float prob = Helpers::normpdf(x_delta,
                                       control_parameter,
                                       position_stdev
  );
  // Printing the returned probability value
  std::cout << prob << "\n";
}


/* Tests the `motion_model` function in `4_coding_the_motion_model.cc`.
 * Returns the motion model probability value, i.e., the likelihood
 * of being at a given position at the current time-step.
 */
void test_motion_model() {
  // Setting the distribution parameters
  float control_stdev = 1.0f;
  float position_stdev = 1.0f;
  // Setting the map parameters
  float movement_per_timestep = 1.0f;  // Number of steps (metres)
  int map_size = 25;                   // Number of discrete positions on map
  // Initialise the landmarks
  std::vector<float> landmark_positions{5, 10, 20};
  // Initialise the prior probability vector
  std::vector<float> priors = initialize_priors(map_size,
                                           landmark_positions,
                                           position_stdev
  );
  // 1. Loop over all pseudo-positions $x_{t}^{(i)}$
  for (int i = 0; i < map_size; i++) {
    float pseudo_position = float(i);
    // 1a-b. Compute the transition and discrete motion probability
    float motion_prob = motion_model(pseudo_position,
                                     movement_per_timestep,
                                     priors,
                                     map_size,
                                     control_stdev
    );
    // Print the resulting motion model probability
    std::cout << pseudo_position << "\t" << motion_prob << "\n";
  }
}


/* Tests the `normpdf` function inside `2_determine_probabilities`.
 * Returns the probability value obtained from the normal distribution
 * parameterised with the given scalar values of (`mu`, `sigma`).
 */
void test_observation_model_probability() {
  // Defining the distribution parameters
  float observation_stdev = 1.0f;
  // The observation measurements
  float observation_measurement_1 = 5.5;
  float observation_measurement_2 = 11.0;
  // The pseudo-range estimates
  float x_delta_1 = 5.0;
  float x_delta_2 = 11.0;
  // Computing the probability of the first pair
  // See: `2022-11-25-Course-3-Localization-Exercises-Part-1.ipynb` 
  std::cout << "Pair 1: [" << observation_measurement_1 << ", " << x_delta_1 << "]\n";
  static float prob1 = Helpers::normpdf(observation_measurement_1,
                                       x_delta_1,
                                       observation_stdev
  );
  std::cout << prob1 << "\n";
  std::cout << "-----------------------" << "\n";
  // Computing the probability of the second pair
  std::cout << "Pair 2: [" << observation_measurement_2 << ", " << x_delta_2 << "]\n";
  static float prob2 = Helpers::normpdf(observation_measurement_2,
                                       x_delta_2,
                                       observation_stdev
  );
  std::cout << prob2 << "\n";
}


/* Tests the `pseudo_range_estimator` function inside `6_get_pseudo_ranges.cc`.
 * Returns the pseudo-range vector containing the landmark distances relative
 * to the given `pseudo_position`.
 */
void test_pseudo_range_estimator() {
  // The landmark positions in 1-D map space
  std::vector<float> landmark_positions{5, 10, 12, 20};
  // The number of discrete positions `x` on the map
  int map_size = 25;
  // Number of metres moved by the vehicle per time-step
  float movement_per_timestep = 1.0f;
  // The standard deviation of the control
  float control_stdev = 1.0f;
  // Compute the pseudo-ranges for each position on the map
  for (int i = 0; i < map_size; i++) {
    float pseudo_position = float(i);
    std::vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions,
                                                              pseudo_position
    );
    // Print the resulting pseudo-range vector values
    if (pseudo_ranges.size()) {
      for (auto val : pseudo_ranges) {
        std::cout << "x: " << i << "\t" << val << "\n";
      }
      std::cout << "-----------------------" << "\n";
    }
    else {
      // No landmarks in front of vehicle
      continue;
    }
  }
}


/* Tests the `observation_model` function in `7_coding_observation_model.cc`.
 * Prints the resulting observation likelihood computed at each pseudo-map
 * position, given the distribution and map parameters and the observation
 * vector, i.e., set of sensor readings obtained from the vehicle.
 */
void test_observation_model() {
  // Define the distribution parameters
  float observation_stdev = 1.0f;
  // Define the map parameters
  int map_size = 25;                  // Number of discrete positions on map
  float dist_max = map_size;          // Maximum position on map
  std::vector<float> landmark_positions{
      5, 10, 15, 20
  };                                  // Ground-truth landmark positions on map
  std::vector<float> observations{
      5.5, 13.0, 15.0
  };                                  // Measurements from vehicle sensor
  // Loop through each pseudo-position on the map
  for (int i = 0; i < map_size; i++) {
    float pseudo_position = float(i);
    // Compute the pseudo-range distances from vehicle to map position
    std::vector<float> pseudo_ranges = pseudo_range_estimator(landmark_positions,
                                                              pseudo_position
    );
    // Compute observation likelihood at this position
    float observation_likelihood = observation_model(landmark_positions,
                                                     observations,
                                                     pseudo_ranges,
                                                     dist_max,
                                                     observation_stdev
    );
    // Print the resulting likelihood value
    std::cout << "Pseudo-position (x): " << pseudo_position << "\n";
    std::cout << observation_likelihood << "\n";
    std::cout << "-----------------------" << "\n";
  }
}


/* Tests the `markov_filter` inside `8_coding_the_full_filter.cc`.
 * Implements the 1D Markov Localization filter based on a set of
 * 1D distribution parameters for the zero-mean control, motion,
 * and observation models. We assume a discretised 1D range map is
 * given, which represents static objects ("landmarks") defined
 * with respect to the ego-vehicle heading. The pseudo-positions
 * are computed as distances from landmarks on the map relative to
 * the current given position. All landmarks with a negative distance
 * i.e., behind the pseudo-position are discarded.
 */
void test_markov_filter() {
  // Run the 1D Markov Localizer 
  markov_filter();
}