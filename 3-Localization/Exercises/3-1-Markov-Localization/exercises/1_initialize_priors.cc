/* ----------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Calculates the 1-D localisation posterior, i.e.,
 *                       the vector of prior probability values for each
 *                       position in the discretised 1-D map relative to the
 *                       ego-vehicle. Here we assume the vehicle starts at one
 *                       of the landmarks with standard deviation +/- 1.0 m.
/* ----------------------------------------------------------------------------


/* Calculates the 1-D location posterior, i.e., the vector of prior probabilty
 * values for each position in the discretised 1-D pose range space (map) with
 * step resolution of 1.0m. The range space is defined relative to the heading
 * of the ego-vehicle. Here we assume the vehicle starts at one of the
 * `n_landmarks` (static objects) with a position std. deviation of +/- 1.0m.
 */
std::vector<float> initialize_priors(
    int map_size,
    std::vector<float> landmark_positions,
    float position_stdev
) {
  // Initialise the prior probabilities
  std::vector<float> priors(map_size, 0.0);
  // Number of total landmarks in map view
  int n_landmarks = landmark_positions.size();
  // Number of neighbours for each landmark
  int n_neighbours =  2 * int(position_stdev);
  // Number of non-zero priors (landmarks plus neighbours)
  int n_pos = n_landmarks + n_neighbours * n_landmarks;
  // The prior probability of vehicle being at a position of interest
  float p_prior = 1.0 / n_pos;
  // Set the non-zero prior probability values
  for (auto i : landmark_positions) {
      // The landmark prior probability
      priors[int(i)] = p_prior;
      // The neighbouring positions' prior probabilities
      priors[int(i - position_stdev)] = p_prior;  // Left of landmark
      priors[int(i + position_stdev)] = p_prior;  // Right of landmark
  }
  return priors;
}
