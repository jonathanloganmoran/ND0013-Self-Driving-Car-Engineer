#include "classifier.h"
#include <math.h>
#include <string>
#include <vector>

using Eigen::ArrayXd;


// Initializes GNB
GNB::GNB() {
  /**
   * TODO: Initialize GNB, if necessary. May depend on your implementation.
   */
  
}
GNB::~GNB() {}


/* Fits the Gaussian Naïve Bayes classifier on the provided data and labels. 
 *
 * @param  data     Nx4 array of observations, each assumed to be a tuple of
 *                  state values [`s`, `d`, `s_dot`, `d_dot`].
 * @param  labels   Nx1 array of labels, each label is a manoeuvre from one of
 *                  the possible strings: ["left", "keep", "right"].  
 */
void GNB::train(
    const std::vector<std::vector<double>>& data, 
    const std::vector<std::string>& labels
) {
  /**
   * TODO: Implement the training function for your classifier.
   */
}


/* Evaluates the give sample on the trained Gaussian Naïve Bayes classifier.
 *
 * @param    sample   1x4 tuple of state values in the form:
 *                    [`s`, `d`, `s_dot`, `d_dot`].
 * @returns  Label of the most-likely prediction for the given sample.
 */
std::string GNB::predict(
    const std::vector<double>& sample
) {
  /**
   * TODO: Complete this function to return your classifier's prediction
   */
  return this->possible_labels[1];
}