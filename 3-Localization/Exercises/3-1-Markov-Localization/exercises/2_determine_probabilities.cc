/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Calculate the 1-D probability density function
 *                       given the value `x` and the parameters `mu`, `sigma`
 *                       of a normal distribution.
 * ----------------------------------------------------------------------------
 */


class Helpers {
public:
  /* Returns the probability function for value `x` assuming a
   * normal distribution parameterised by (`mu`, `sigma`).
   */
  static float normpdf(float x, float mu, float std) {
    // Distribution parameters must be scalars
    // and std must be positive
    assert(std > 0.0);
    // Compute the probability function
    return (kOneOverSqrt2Pi / std) * exp(-0.5 * pow((x - mu) / std, 2));
  }
};