/* ------------------------------------------------------------------------------
 * Lesson "4.4: Prediction"
 * Authors     : Mahni Shayganfar.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Implements the Gaussian Naïve Bayes (GNB) classifier.
 * ----------------------------------------------------------------------------
 */

#include "classifier.h"


// Initialises GNB with ground-truth labels
GNB::GNB()
  : possible_labels({"left", "keep", "right"}) {
  /*** Initialise the data tables ***/
  for (const auto& label : this->possible_labels) {
    this->class_priors.emplace(
        std::make_pair<const std::string&, double>(
            label, 0.0
        )
    );
    this->data_table.emplace(
        std::make_pair<const std::string&, std::vector<std::vector<double>>>(
            label, std::vector<std::vector<double>>()
        )
    );
    this->conditional_probabilities.emplace(
        std::make_pair<const std::string&, std::vector<std::vector<double>>>(
            label, std::vector<std::vector<double>>()
        )
    );
  }
}
// Implements the destructor
GNB::~GNB() {}


/* Computes the frequency of each class.
 *
 * The number of occurrences in `labels` of each class from `possible_labels`
 * is counted and stored in a map instance such that the key is the class
 * label and the value is the number of occurrences of the respective class
 * in the given `labels` vector.
 * 
 * @param    labels         String-valued labels of every training example.
 * @returns  class_counts   Frequency dict for each class in `possible_labels`.
 */
std::map<std::string, double> GNB::unique_occurrences(
    const std::vector<std::string>& labels
) {
  // Initialise the map using the classes as the keys
  std::map<std::string, double> class_counts;
  for (const auto& label : this->possible_labels) {
    // Initialise the count of the class label
    // NOTE: `make_pair` is used to insert `const`-valued key
    class_counts.emplace(
        std::make_pair<const std::string&, double>(
            label, 0.0
        )
    );
  }
  // Compute the frequency of each class in the training labels
  for (const auto& label : labels) {
    // Get the current class count
    auto cls_iter = class_counts.find(label);
    // Increment the class count (frequency)
    cls_iter->second += 1;
  }
  return class_counts;
}


/* Computes the per-class mean and standard deviation for each feature. 
 *
 * Tabularises the data for each class, then computes the per-class statistics
 * for each feature. The resulting `feature_statistics` map contains Gaussian
 * parameters `mu` and `sigma` computed w.r.t. each feature of each class in
 * the training `data`.
 * 
 * @param  data     Training data, each sample is a vector of state values.
 * @param  labels   Set of corresponding class labels for the training `data`.
 */
void GNB::compute_statistics(
    const std::vector<std::vector<double>>& data,
    const std::vector<std::string>& labels
) {
  /*** Construct the tabular dataset ***/
  // Initialise vector of Gaussian distribution parameters
  std::vector<Stats> mu_sigma;
  if (labels.size() != data.size()) {
    throw "ERROR: Labels and data are of different lengths";
  }
  for (int i = 0; i < data[0].size(); i++) {
    // Initialise each feature in feature vector with zero distribution values
    mu_sigma.push_back(Stats{0.0, 0.0});
  }
  for (const auto& label : this->possible_labels) {
    // Initialise each key in the data table
    this->feature_statistics.emplace(
        std::make_pair<const std::string&, std::vector<Stats>>(
            label, std::vector<Stats>{mu_sigma}
        )
    );
  }
  // Compute the class frequencies
  std::map<std::string, double> freq = unique_occurrences(labels);
  /*** Compute the per-class mean of each feature ***/
  for (int i = 0; i < labels.size(); i++) {
    data_table[labels[i]].push_back(data[i]);
    for (int idx_feat = 0; idx_feat < data[i].size(); idx_feat++) {
      this->feature_statistics[labels[i]][idx_feat].mu += data[i][idx_feat];
    }
  }
  for (const auto& label : this->possible_labels) {
    for (int idx_feat = 0; idx_feat < data[0].size(); idx_feat++) {
      this->feature_statistics[label][idx_feat].mu /= freq[label];
    }
  }
  /*** Compute the per-class standard deviation of each feature ***/
  for (const auto& label : this->possible_labels) {
    for (const auto& cls_data : data_table[label]) {
      for (int idx_feat = 0; idx_feat < cls_data.size(); idx_feat++) {
        this->feature_statistics[label][idx_feat].sigma += std::pow(
            cls_data[idx_feat] - this->feature_statistics[label][idx_feat].mu,
            2
        );
      }
    }
  }
  for (const auto& label : this->possible_labels) {
    for (int idx_feat = 0; idx_feat < data[0].size(); idx_feat++) {
      this->feature_statistics[label][idx_feat].sigma /= freq[label];
      this->feature_statistics[label][idx_feat].sigma = std::sqrt(
          this->feature_statistics[label][idx_feat].sigma
      );
    }
  }
  // Update the class priors
  for (const auto& label : this->possible_labels) {
    // Compute the current class prior probability
   class_priors[label] = freq[label] / labels.size();
  }
}


/* Evaluates the Gaussian probability density function for the given values.
 *
 * @param   x               Current feature value to evaluate.
 * @param   mu              Mean value of the feature / label combination.
 * @param   sigma           Standard deviation of the feature / label combination.
 * @returns p_conditional   Resulting conditional probability value.
 */
double GNB::gaussian_pdf(
    double x,
    double mu,
    double sigma
) {
  double sigma_squared = sigma * sigma;
  double p_conditional = 1.0 / std::sqrt(2 * M_PI * sigma_squared);
  p_conditional *= std::exp(
      -std::pow((x - mu), 2) / (2.0 * sigma_squared)
  );
  return p_conditional;
}


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
  /*** Fit the per-class Gaussian distribution for each feature ***/
  compute_statistics(data, labels);
}


/* Evaluates the given sample on the trained Gaussian Naïve Bayes classifier.
 *
 * @param    sample   1x4 tuple of state values in the form:
 *                    [`s`, `d`, `s_dot`, `d_dot`].
 * @returns  Label of the most-likely prediction for the given sample.
 */
std::string GNB::predict(
    const std::vector<double>& sample
) {
  std::map<std::string, double> y_pred;
  for (const auto& label : this->possible_labels) {
    double p_posterior = this->class_priors[label]; 
    // Compute the per-class conditional probabilities
    for (int idx_feat = 0; idx_feat < sample.size(); idx_feat++) {
      double p_cond = gaussian_pdf(
          sample[idx_feat],
          this->feature_statistics[label][idx_feat].mu,
          this->feature_statistics[label][idx_feat].sigma
      );
      p_posterior *= p_cond;
    }
    y_pred[label] = p_posterior;
  }
  // Compute the most-likely class (i.e., the argmax)
  return std::max_element(
      y_pred.begin(),
      y_pred.end(),
      [](const std::pair<std::string, double>& a, const std::pair<std::string, double> b) {
          return a.second < b.second; 
      })->first;
}