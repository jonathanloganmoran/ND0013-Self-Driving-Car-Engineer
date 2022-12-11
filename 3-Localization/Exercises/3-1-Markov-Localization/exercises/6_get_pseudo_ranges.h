/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the `pseudo_range_estimator` function
 *                       which returns the landmark range estimates that are
 *                       in front of a given pseudo-position.
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
);