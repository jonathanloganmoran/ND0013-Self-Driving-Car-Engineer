/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the `observation_model` function which
 *                       returns the likelihood value for a given set of 2D
 *                       sensor `observations` and 1D `pseudo_ranges` vector.
 * ----------------------------------------------------------------------------
 */


#include <vector>


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
);