/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the Lesson 3.1 Helper functions,
 *                       i.e., the 1-D probability density function and a
 *                       function to return a normalised vector using the
 *                       mean-value normalisation technique.
 * ----------------------------------------------------------------------------
 */


#include <cassert>
#include <math.h>
#include <vector>


// Define the normalisation term
constexpr static float kOneOverSqrt2Pi = 1.0 / sqrt(2 * M_PI);

class Helpers {
public:
  // Computes the probability function at `x`
  // given the normal distribution parameterised by (`mu`, `sigma`) 
  static float normpdf(
      float x,
      float mu,
      float std
  );

  // Returns the re-scaled / normalised input vector using
  // mean-value normalisation.
  static std::vector<float> normalize_vector(
      std::vector<float> input_vector
  );
};