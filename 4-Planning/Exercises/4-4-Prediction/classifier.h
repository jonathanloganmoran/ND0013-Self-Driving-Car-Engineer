#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include "Dense"            // Eigen::ArrayXd
#include <string>
#include <vector>

using Eigen::ArrayXd;


/* The Gaussian Naïve Bayes classifier.
 *
 * Predicts the most-likely vehicle manoeuvre given a state vector of values:
 *    [`s`, `d`, `s_dot`, `d_dot`],
 * where each is double of position and velocity components defined in the
 * Frenet coordinate space.
 * 
 * The Guassian Naïve Bayes classifier exploits the Bayes rule assuming the
 * data is normally-distributed in order to predict the most-likely manoeuvre
 * from the set of three possible manoeuvres:
 *    ["left", "keep" "right"],
 * as defined in the `possible_labels` vector here.
 * 
 * @param  possible_labels  Set of possible vehicle manoeuvres to predict. 
 */
class GNB {
 public:
  GNB();
  virtual ~GNB();
  // Fits the Gaussian Naïve Bayes classifier on the provided data and labels
  void train(
      const std::vector<std::vector<double>>& data,
      const std::vector<std::string>& labels
  );
  // Evaluates the give sample on the trained Gaussian Naïve Bayes classifier
  string predict(const vector<double> &sample);
  // Possible manoeuvres to predict with the GNBC
  std::vector<std::string> possible_labels = {
      "left",
      "keep",
      "right"
  };
};


#endif  // CLASSIFIER_H