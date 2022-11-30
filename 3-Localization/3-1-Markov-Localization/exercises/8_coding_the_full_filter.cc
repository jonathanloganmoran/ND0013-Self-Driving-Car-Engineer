/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Defines the `markov_filter` which performs 1D Markov
 *                       Localization filtering over a set of 2D observation
 *                       vectors collected at each time-step. Here we assume
 *                       the control / motion / observation models are zero-mean
 *                       1D Gaussians with known standard deviation. The map
 *                       is given as 1D set of range values defined for static
 *                       objects ("landmarks") assumed to be on the same axis
 *                       as the ego-vehicle heading.
 * ----------------------------------------------------------------------------
 */


/* Performs 1D Markov Localization filtering over a set of
 * 2D observation vectors collected at each time-step.
 * The control / motion models are assumed to be zero-mean
 * distributions parameterised with known standard deviations.
 * The discretised 1D range map represents a set of static objects
 * ("landmarks") defined with respect to the ego-vehicle heading.
 * The pseudo-positions are computed as distances from a given map
 * position to the known landmarks. All landmarks with a negative
 * distance to (i.e., behind) the pseudo-position are discarded.
 */
void markov_filter() {  
  // Distribution parameters
  float control_stdev = 1.0f;
  float position_stdev = 1.0f;
  float observation_stdev = 1.0f;
  // Movement model parameters
  // The number of map positions moved by the vehicle per time-step
  float movement_per_timestep = 1.0f;
  // Map parameters
  int map_size = 25;            // Total num. discrete positions on map
  float dist_max = map_size;    // Last position on map
  std::vector<float> landmark_positions{3, 9, 14, 23};
  // Observation vector
  std::vector<std::vector<float>> sensor_obs{
      {1, 7, 12, 21}, {0, 6, 11, 20}, {5, 10, 19},
      {4, 9, 18}, {3, 8, 17}, {2, 7, 16}, {1, 6, 15},
      {0, 5, 14}, {4, 13}, {3, 12}, {2, 11}, {1, 10},
      {0,9}, {8}, {7}, {6}, {5}, {4}, {3}, {2},
      {1}, {0}, {}, {}, {}
  };
  // Initialise the prior probabilities
  std::vector<float> priors = initialize_priors(map_size,
                                                landmark_positions,
                                                position_stdev
  );
  // UNCOMMENT TO PRINT THE INITIALISED PRIOR PROBABILITIES
  // std::cout << "-----------PRIORS INIT--------------" << "\n";
  // for (auto p : priors) {
  //   std::cout << p << "\n";  
  // }
  // Initialise the posterior probabilities
  std::vector<float> posteriors(map_size, 0.0);
  // Declare the observations vector
  std::vector<float> observations;
  // Obtain the number of time-steps in this sensor observations vector 
  int time_steps = sensor_obs.size();

  // 0. Loop over all observation time-steps
  for (int t = 0; t < time_steps; ++t) {
    // UNCOMMENT TO PRINT TIME-STEP AND PROCESS FLOW-CHART
    // std::cout << "---------------TIME STEP---------------" << "\n";
    // std::cout << "t = " << t << "\n";
    // std::cout << "-----Motion----------OBS---------------PRODUCT--" << "\n";
      
    // 1. Extract the sensor observations from this time-step
    if (!sensor_obs[t].empty()) {
      observations = sensor_obs[t];
    }
    else {
      // No sensor observations from this time-step
      // Initialise observations vector with map size
      observations = {float(dist_max)};
    }
      
    // 2. Loop over each pseudo-position $x_{t}^{(i)}$
    for (int i = 0; i < map_size; ++i) {
      float pseudo_position = float(i);
      // 2a. Compute the motion model probability
      float p_motion = motion_model(pseudo_position,
                                    movement_per_timestep,
                                    priors,
                                    map_size,
                                    control_stdev
      );
      // 2b. Compute the corresponding pseudo-range vectors
      std::vector<float> pseudo_ranges = pseudo_range_estimator(
          landmark_positions,
          pseudo_position
      );
      // 2c. Compute the posterior probability for the ith time-step
      // First, obtain the observation model likelihood
      float observation_likelihood = observation_model(landmark_positions,
                                                       observations,
                                                       pseudo_ranges,
                                                       dist_max,
                                                       observation_stdev
      );
      // Then, compute the ith posterior as an intersection of events
      float p_posterior = p_motion * observation_likelihood;
      // Set the ith posterior probability to the computed value
      // at this pseudo-map position `i`
      posteriors[i] = p_posterior;
      // UNCOMMENT TO PRINT THE ITH POSTERIOR PROBABILITY VALUE
      // std::cout << p_motion << "\t" << observation_likelihood << "\t" 
      //     << "\t"  << p_posterior << "\n";   
    }   
    // UNCOMMENT TO PRINT ALL POSTERIOR PROBABILITY VALUES
    // std::cout << "----------RAW---------------" << "\n";
    // for (auto p : posteriors) {
    //   std::cout << p << "\n";
    // }
      
    // 3. Normalise the posterior probability values
    static std::vector<float> posteriors_normalized = Helpers::normalize_vector(
       posteriors
    );
    // UNCOMMENT TO REVERT BACK TO NON-NORMALISED POSTERIOR VECTOR  
    // static std::vector<float> posteriors_normalized = posteriors;
    // UNCOMMENT TO PRINT THE NORMALISED POSTERIORS
    std::cout << "----------NORMALIZED---------------" << "\n";
    for (auto p_n : posteriors_normalized) {
      std::cout << p_n << "\n";
    }
      
    // 4. Update the prior probabilities with the posteriors
    priors = posteriors_normalized;
  } // 5. Repeat for all observation time-steps until none remain
}