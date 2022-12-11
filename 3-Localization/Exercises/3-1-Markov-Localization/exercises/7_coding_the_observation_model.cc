/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Defines the `observation_model` which returns the
 *                       likelihood value for a given set of `observations` and
 *                       `pseudo_ranges`.
 * ----------------------------------------------------------------------------
 */


/* Returns the observation probability, i.e., the likelihood
 * value for a given set of `observations` and `pseudo_ranges`.
 * Here we assume the `pseudo_ranges` vector is sorted in ascending
 * order, such that the minimum landmark distance relative to the vehicle
 * occurs at the first index, i.e., `pseudo_ranges[0]`. If no landmarks
 * exist relative to the vehicle heading (i.e., `psuedo_ranges` is empty),
 * then the distance is initialised to a very large number (i.e., `infinity`).
 */
float observation_model(
    std::vector<float> landmark_positions,
    std::vector<float> observations,
    std::vector<float> pseudo_ranges,
    float distance_max,
    float observation_stdev
) {
  // Initialise the observation likelihood value
  float observation_likelihood = 1.0f;
  // 1. Loop over all observations
  for (auto z_i : observations) {
    // 1b. Extract the minimum distance from sorted `pseudo_ranges`
    float dist_min;
    // 1a. Check if pseudo-range vector is non-empty
    if (!pseudo_ranges.empty()) {
      // Assuming `pseudo_ranges` is sorted in ascending order
      dist_min = pseudo_ranges[0];
      // Remove the minimum distance from the vector
      pseudo_ranges.erase(pseudo_ranges.begin());
    }
    else {
      // 1c. No observations in front of vehicle
      dist_min = std::numeric_limits<const float>::infinity();
    }
    // 1c. Compute the probability using `normpdf`
    float p_obs = Helpers::normpdf(z_i,
                                   dist_min,
                                   observation_stdev
    );
    // Update the observation likelihood for this observation
    // using the product rule (i.e., intersection of events)  
    observation_likelihood *= p_obs;
  }
  return observation_likelihood;
}