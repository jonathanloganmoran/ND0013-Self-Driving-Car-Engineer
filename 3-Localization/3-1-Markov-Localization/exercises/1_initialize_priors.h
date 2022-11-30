/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for `initialize_priors`. Computes the 1-D
 *                       localisation posterior probability vector.
 * ----------------------------------------------------------------------------
 */


#include <iostream>
#include <vector>


/* Returns the `priors` vector with corresponding probability values.
 * Here, assume vehicle starts at a landmark with +/- 1.0 m position stdev.
 */
std::vector<float> initialize_priors(
    int map_size,
    std::vector<float> landmark_positions,
    float position_stdev
);
