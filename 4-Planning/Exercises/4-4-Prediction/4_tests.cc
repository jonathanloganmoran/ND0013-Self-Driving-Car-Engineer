/* ------------------------------------------------------------------------------
 * Lesson "4.4: Prediction"
 * Authors     : Mahni Shayganfar.
 *
 * Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Tests for Extracurricular Exercise 4.4.1.
 * ----------------------------------------------------------------------------
 */

#include "classifier.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>


/* Parses the given `.csv` file and returns the state vector values.
 *
 * Each row in the input `.csv` file (located at `file_path`) is expected to
 * contain a set of four comma-separated floating-point values corresponding to
 * the values in each state vector. This has the form:
 *    $[s, d, \dot{s}, \dot{d}]$,
 * where the vehicle coordinates in the Frenet reference frame are given by
 * the first two values $(s, d)$. The rate-of-change of each are given by the
 * next two values $\dot{s}, \dot{d}$. These values are stored in a vector
 * and appended to the returned vector of state(s) in the input file.
 * 
 * @param    file_path   Path to file of comma-delimited state vector values.
 * @returns  states_out  Vector of state vectors parsed from the input.
 */
std::vector<std::vector<double>> load_state_data(
    std::string file_path
) {
  // Load the input file
  std::ifstream in_states_(file_path.c_str(), ifstream::in);
  // Instantiate the output vector
  std::vector<std::vector<double>> states_out;
  // Iterate over each line of the input file
  std::string line;
  while (std::ifstream::getline(in_states_, line)) {
    std::istringstream iss(line);
    std::vector<double> coordinates;
    std::string token;
    while (std::ifstream::getline(iss, token, ',')) {
      coordinates.push_back(stod(token));
    }
    states_out.push_back(coordinates);
  }
  return states_out;
}


/* Parses the given `.csv` file and returns the class labels.
 *
 * Each row in the input `.csv` file (located at `file_path`) is expected to
 * contain a string of characters corresponding to the class label of the
 * respective entry in the state vector data file. In other words, the row
 * number of the label given here belongs to the state vector given at the
 * same row number in the respective state vector file.
 * 
 * @param    file_path   Path to file of comma-delimited class labels.
 * @returns  labels_out  Vector of class labels parsed from the input.
 */
std::vector<std::string> load_labels(
    std::string file_path
) {
  // Load the input file
  std::ifstream in_labels_(file_path.c_str(), std::ifstream::in);
  // Instantiate the output vector
  std::vector<std::string> labels_out;
  // Iterate over each line of the input file
  std::string line;
  while (std::ifstream::getline(in_labels_, line)) {
    std::istringstream iss(line);
    std::string label;
    iss >> label;
    labels_out.push_back(label);
  }
  return labels_out;
}


/* Evaluates the performance of the Gaussian Näive Bayes (GNB) classifier.
 *
 * The GNB classifier is evaluated on the provided dataset in `nd013_pred_data`
 * folder. The training / testing datasets are stored as comma-separated files
 * containing the behaviour of the vehicles along a three-lane highway.
 * Each of the three highway lanes are assumed to have a width of four metres.
 * The coordinates of the vehicles in each lane are given in the Frenet
 * coordinate reference system $(s, d)$ where $s$ is the value of the vehicle
 * along the $x$-axis, and $d$ is the value of the vehicle along the $y$-axis.
 * The Frenet coordinate frame is defined w.r.t. the centre of each lane.
 * Each row in the `.csv` dataset is a comma-delimited set of state vector
 * values in the form:
 *      $[s, d, \dot{s}, \dot{d}]$,
 * which are the Frenet coordinates and rate-of-change values along each axis
 * of a single vehicle at the point in time in which the values were recorded.
 * Each observation in both the training and the test sets contains a set of
 * corresponding ground-truth labels, where each label belongs to a set of
 * possible vehicle manoeuvres (`possible_labels` i.e., the class labels):
 *      $["left", "keep", "right"]$.
 *
 * The GNB is fit on the training data, which in other words, forms a set
 * of Gaussians parameterised by the mean and standard deviation of each
 * label-feature pair. The prior probability is also computed for each
 * label-feature pair.
 * 
 * During the prediction step, the most-likely vehicle manoeuvre from the
 * set of `possible_labels` is estimated for the corresponding observation
 * from the test set. In order to compute this, the argmax across the
 * probabilities formed by the Gaussian probability distribution function
 * is computed. The evaluation function here (`test_gnb_classifier`) parses
 * the datasets, fits the `GNB` classifier on the training data, performs the
 * prediction over the test data, and computes the final accuracy of the model
 * as the number of correct predictions divided by the total number of test
 * set observations. 
 */
void test_gnb_classifier() {
  /*** Reading in the training dataset ***/
  std::string train_data_file_path = "../train_states.txt";
  std::string train_labels_file_path = "../train_labels.txt";
  std::vector<std::vector<double>> X_train = load_state_data(
      train_data_file_path
  );
  std::vector<std::string> y_train = load_labels(
      train_labels_file_path
  );
  /*** Reading in the test dataset ****/
  std::string test_data_file_path = "../test_states.txt";
  std::string test_labels_file_path = "../test_labels.txt";
  std::vector<std::vector<double>> X_test = load_state_data(
      test_data_file_path
  );
  std::vector<std::string> y_test = load_labels(
      test_labels_file_path
  );
  /*** Printing dataset statistics ***/
  std::cout << "`X_train` samples: " << X_train.size() << "\n";
  std::cout << "`y_train` labels: " << y_train.size() << "\n";
  std::cout << "`X_test` samples: " << X_test.size() << "\n";
  std::cout << "`y_test` labels: " << y_test.size() << "\n";
  /*** Performing classification with GNB ***/
  GNB gnb = GNB();
  gnb.train(X_train, y_train);
  /*** Computing the prediction accuracy score ***/
  int n_correct = 0.0;
  for (int i = 0; i < X_test.size(); i++) {
    std::vector<double> x_test_i = X_test[i];
    std::string predicted = gnb.predict(x_test_i);
    if (predicted.compare(y_test[i]) == 0) {
      n_correct += 1;
    }
  }
  float acc = float(n_correct) / y_test.size();
  std::cout << "GNB accuracy: " << (100.0 * acc) << "%" << "\n";
}


int main() {
  // Extracurricular Exercise 4.4.1: Gaussian Naïve Bayes (GNB) classifier
  test_gnb_classifier();
}