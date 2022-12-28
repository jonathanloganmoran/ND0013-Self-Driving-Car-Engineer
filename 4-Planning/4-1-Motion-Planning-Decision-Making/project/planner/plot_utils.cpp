/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file plot_utils.cpp
 **/

#include "plot_utils.h"

namespace plt = matplotlibcpp;

namespace utils {

void plotWaypoints(
    std::vector<carla::SharedPtr<carla::client::Waypoint>> waypoints) {
  auto n = waypoints.size();  // number of data points
  std::vector<float> x(n), y(n);
  for (size_t i = 0; i < n; i++) {
    auto t = waypoints[i]->GetTransform();
    x.at(i) = t.location.x;
    y.at(i) = t.location.y;
  }

  // plot() takes an arbitrary number of (x,y,format)-triples. x must be
  // iterable (that is, anything providing begin(x) and end(x)), y must either
  // be callable (providing operator() const) or iterable.
  plt::plot(x, y, "ro");

  // show plots
  plt::show();
}

void plot_goals(std::vector<State> goals) {
  auto n = goals.size();  // number of data points
  std::vector<float> x(n), y(n);
  for (size_t i = 0; i < n; i++) {
    x.at(i) = goals[i].location.x;
    y.at(i) = goals[i].location.y;
  }

  // Clear previous plot
  // plt::clf();

  // plot() takes an arbitrary number of (x,y,format)-triples. x must be
  // iterable (that is, anything providing begin(x) and end(x)), y must either
  // be callable (providing operator() const) or iterable.
  plt::plot(x, y, "ro");

  // show plots
  // plt::show();
  // Display plot continuously
  plt::pause(0.1);
}

void plot_spiral(std::vector<carla::geom::Transform> spiral) {
  auto n = spiral.size();  // number of data points
  std::vector<float> x(n), y(n);
  for (size_t i = 0; i < n; i++) {
    x.at(i) = spiral[i].location.x;
    y.at(i) = spiral[i].location.y;
    // if (i % 1 == 0) {
    //   // Clear previous plot
    //   plt::clf();
    //   // Plot line from given x and y data. Color is selected automatically.
    //   plt::plot(x, y, "ro");

    //   // Plot a line whose name will show up as "Waypoints" in the legend.
    //   // plt::named_plot("Waypoints", 10, 10);

    //   // Add graph title
    //   plt::title("Spiral");
    //   // Enable legend.
    //   // plt::legend();
    //   // Display plot continuously
    //   plt::pause(0.01);
  }
  // Clear previous plot
  // plt::clf();

  // plot() takes an arbitrary number of (x,y,format)-triples. x must be
  // iterable (that is, anything providing begin(x) and end(x)), y must either
  // be callable (providing operator() const) or iterable.
  plt::plot(x, y, "ro");

  // show plots
  // plt::show();
  // Display plot continuously
  plt::pause(0.1);
}

void plotWaypointsAnimation(
    std::vector<carla::SharedPtr<carla::client::Waypoint>> waypoints) {
  // Clear previous plot
  plt::clf();
  auto n = waypoints.size();  // number of data points
  std::vector<float> x(n), y(n);
  for (size_t i = 0; i < n; i++) {
    auto t = waypoints[i]->GetTransform();
    x.at(i) = t.location.x;
    y.at(i) = t.location.y;

    if (i % 10 == 0) {
      // Clear previous plot
      plt::clf();
      // Plot line from given x and y data. Color is selected automatically.
      plt::plot(x, y, "r-");

      // Plot a line whose name will show up as "Waypoints" in the legend.
      // plt::named_plot("Waypoints", 10, 10);

      // Add graph title
      plt::title("Waypoints");
      // Enable legend.
      plt::legend();
      // Display plot continuously
      plt::pause(0.01);
    }
  }
}

template <typename T>
void plotAnimation(const std::vector<T> xs, const std::vector<T> ys) {
  // Check for consistency
  static_assert(xs.size() == ys.size());

  // Clear previous plot
  plt::clf();

  auto n = xs.size();  // number of data points
  for (size_t i = 0; i < n; i++) {
    if (i % 10 == 0) {
      // Clear previous plot
      plt::clf();
      // Plot line from given x and y data. Color is selected automatically.
      plt::plot(xs, ys, "r-");

      // Plot a line whose name will show up as "log(x)" in the legend.
      // plt::named_plot("Waypoints", x[i], y[i]);

      // Add graph title
      plt::title("Waypoints");
      // Enable legend.
      plt::legend();
      // Display plot continuously
      plt::pause(0.01);
    }
  }
}

}  // namespace utils