/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Returns the `pseudo_ranges` vector, i.e., the landmark
 *                       range estimates in front of the given pseudo-position.
 *                       Here `pseudo_position` is given in 1D map coordinates.
 * ----------------------------------------------------------------------------
 */


#include <algorithm>
#include <vector>


/* Returns the pseudo-range estimates for a given set
* of landmark positions. The positions are defined
* in 1-D global map coordinates with respect to the
* heading of the ego-vehicle (i.e., the forward motion).
*/
std::vector<float> pseudo_range_estimator(
    std::vector<float> landmark_positions,
    float pseudo_position
) {
  // Initialise the pseudo-observation vector and distance
  std::vector<float> pseudo_ranges(0.0, landmark_positions.size());
  float dist = 0.0;
  // 1. Loop over landmarks
  for (auto l_pos : landmark_positions) {
    // 1a. Compute the distance between position and landmark
    dist = l_pos - pseudo_position;
    if (dist > 0.0) {
      // Landmark is in front of pseudo-position, append to list
      pseudo_ranges.push_back(dist);
    }
    else {
      // Skip the landmark, could be behind vehicle
      continue;
    }
  }
  // Sort the resulting vector in ascending order
  std::sort(pseudo_ranges.begin(), pseudo_ranges.end());
  return pseudo_ranges;
}