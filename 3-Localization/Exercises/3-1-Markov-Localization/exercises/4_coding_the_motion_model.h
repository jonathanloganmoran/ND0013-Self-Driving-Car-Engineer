/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the 1D motion model which returns the
 *                       probability of being at a given position in time.
 * ----------------------------------------------------------------------------
 */


#include "2_determine_probabilities.h"
#include <vector>


/* Returns the motion model probability value, i.e., the likelihood
 * of being at a given position at the current time-step.
 */
float motion_model(
    float pseudo_position,
    float movement,
    std::vector<float> priors,
    int map_size,
    int control_stdev
);