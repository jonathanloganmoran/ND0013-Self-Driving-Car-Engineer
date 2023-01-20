/* ------------------------------------------------------------------------------
 * Lesson "4.4: Prediction"
 * Authors     : Mahni Shayganfar.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the Gaussian Naïve Bayes classifier.
 * ----------------------------------------------------------------------------
 */

#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <map>
#include <utility>


/* The Gaussian Naïve Bayes classifier.
 *
 * Predicts the most-likely vehicle manoeuvre given a state vector of values:
 *    [`s`, `d`, `s_dot`, `d_dot`],
 * where each is double of position and velocity components defined in the
 * Frenet coordinate space.
 * 
 * The Guassian Naïve Bayes classifier exploits the Bayes rule assuming the
 * data in each class (the state vector values) is normally-distributed to
 * predict the most-likely manoeuvre from the set of three possible manoeuvres:
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
  std::string predict(
      const std::vector<double>& sample
  );
  // Computes the number of occurrences of the class labels in the vector
  std::map<std::string, double> unique_occurrences(
      const std::vector<std::string>& labels
  );
  // Computes the Gaussian distribution statistics
  void compute_statistics(
      const std::vector<std::vector<double>>& data,
      const std::vector<std::string>& labels
  );
  // Evaluate the Gaussian probability density function
  double gaussian_pdf(
    double x,
    double mu,
    double sigma
  );
  // Stores the distribution parameters for each feature
  struct Stats {
    // Mean value
    double mu;
    // Standard deviation
    double sigma;
  };
  // Possible manoeuvres to predict with the GNBC
  // NOTE: should initialise with member initialisation list
  std::vector<std::string> possible_labels;
  /*** Data tables ***/
  // NOTE: should initialise these inside constructor { body }
  // Per-class prior probabilities
  std::map<std::string, double> class_priors;
  // Data table to store per-class examples
  std::map<std::string, std::vector<std::vector<double>>> data_table;
  // Conditional probabilities for all labels w.r.t. the feature values
  std::map<std::string, std::vector<std::vector<double>>> conditional_probabilities;
  // Distribution parameters for each feature
  std::map<std::string, std::vector<Stats>> feature_statistics;
};


#endif  // CLASSIFIER_H