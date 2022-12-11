/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the 1D motion model function which returns
 *                       the probability of being at a given position in time.
 * ----------------------------------------------------------------------------
 */


/* Returns the motion model probability value, i.e., the likelihood
 * of being at a given position at the current time-step.
 */
float motion_model(
    float pseudo_position,
    float movement,
    std::vector<float> priors,
    int map_size,
    int control_stdev
) {
  // Initialise the position probability
  float prob_position = 0.0f;
  // 1. Loop over all positions in state space (performing convolution)
  for (int j = 0; j < map_size; j++) {
    float pseudo_position_next = float(j);
    // 1a. Compute the transition probability
    float dist = pseudo_position - pseudo_position_next;
    float prob_transition = Helpers::normpdf(dist,
                                             movement,
                                             control_stdev
    );
    // Estimate the motion model probability with the product rule
    prob_position += prob_transition * priors[j];
  }
  // Return the motion model probability value
  return prob_position;
}