/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file utils.cpp
 **/

#include "utils.h"

#include <cmath>

namespace utils {

double distance(const double x1, const double y1, const double x2,
                const double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double distance(const PathPoint point1, const PathPoint point2) {
  return sqrt((point2.x - point1.x) * (point2.x - point1.x) +
              (point2.y - point1.y) * (point2.y - point1.y) +
              (point2.z - point1.z) * (point2.z - point1.z));
}

double magnitude(carla::geom::Vector3D vector) {
  return sqrt((vector.x) * (vector.x) + (vector.y) * (vector.y) +
              (vector.z) * (vector.z));
}

// For converting back and forth between radians and degrees.
double deg2rad(double x) { return x * M_PI / 180; }
double rad2deg(double x) { return x * 180 / M_PI; }

double random_noise(double start, double end) {
  // Will be used to obtain a seed for the random number engine
  std::random_device rd;

  // Standard mersenne_twister_engine seeded with rd()
  std::mt19937 gen(rd());

  std::uniform_real_distribution<> dis(start, end);

  return dis(gen);
}

double evaluate(std::vector<double> coefficients, double t) {
  ////////////cout << "evaluate - Coeff: " << coefficients.size() << endl;
  if (t == 0) {
    return coefficients[0];
  }

  double total = 0.0;
  for (size_t i = 0; i < coefficients.size(); i++) {
    total += coefficients[i] * pow(t, i);
  }
  return total;
}

std::vector<double> differentiate(std::vector<double> coefficients) {
  /*
  Calculates the derivative of a polynomial and returns
  the corresponding coefficients.
  */
  std::vector<double> derivative_coeff;
  for (size_t i = 1; i < coefficients.size(); i++) {
    derivative_coeff.push_back(coefficients[i] * i);
    ////////////cout << "diff - Coeff[" << i-1 << "] = " <<
    /// derivative_coeff[i-1] << endl;
  }

  return derivative_coeff;
}

std::vector<double> evaluate_f_and_N_derivatives(
    std::vector<double> coefficients, double t, int N) {
  std::vector<double> values;
  values.push_back(evaluate(coefficients, t));
  std::vector<double> d_coeff = differentiate(coefficients);
  for (int i = 1; i <= N; i++) {
    values.push_back(evaluate(d_coeff, t));
    d_coeff = differentiate(d_coeff);
  }
  return values;
}

double logistic(double x) {
  /*
  A function that returns a value between 0 and 1 for x in the
  range[0, infinity] and -1 to 1 for x in the range[-infinity, infinity].

  Useful for cost functions.
  */

  return 2.0 / (1 + exp(-x)) - 1.0;
}

double logistic2(double x) {
  /*
  A function that returns a value between 0 and 1 for x in the
  range[0, infinity] and -1 to 1 for x in the range[-infinity, infinity].

  Useful for cost functions.
  */

  return 1.0 / (1 + exp(-x));
}

void solve_quadratic(double a, double b, double c, double& x1, double& x2) {
  double discriminant = b * b - 4 * a * c;

  if (discriminant > 0) {
    x1 = (-b + sqrt(discriminant)) / (2 * a);
    x2 = (-b - sqrt(discriminant)) / (2 * a);
  }

  else if (discriminant == 0) {
    x1 = (-b + sqrt(discriminant)) / (2 * a);
    x2 = x1;
  } else {
  }
}
std::array<double, 2> solve_quadratic(double a, double b, double c) {
  double x1, x2;

  double discriminant = (b * b) - (4 * a * c);

  if (discriminant > 0) {
    x1 = (-b + sqrt(discriminant)) / (2 * a);
    x2 = (-b - sqrt(discriminant)) / (2 * a);
  }

  else if (discriminant == 0) {
    x1 = (-b + sqrt(discriminant)) / (2 * a);
    x2 = x1;
  }

  else {
    return {};
  }
  return {x1, x2};
}

float keep_angle_range_rad(float angle, double lower_limit,
                           double upper_limit) {
  if (angle < lower_limit) {
    angle += (2 * M_PI);
  } else if (angle > upper_limit) {
    angle -= (2 * M_PI);
  }
  return angle;
}

float keep_angle_range_deg(float angle, double lower_limit,
                           double upper_limit) {
  if (angle < lower_limit) {
    angle += (360);
  } else if (angle > upper_limit) {
    angle -= (360);
  }
  return angle;
}

template <typename T>
std::vector<T> linspace(T a, T b, size_t N) {
  T h = (b - a) / static_cast<T>(N - 1);
  std::vector<T> xs(N);
  typename std::vector<T>::iterator x;
  T val;
  for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
    *x = val;
  }
  return xs;
}

std::vector<float> linspace2(float a, float b, size_t N) {
  float h = (b - a) / static_cast<float>(N - 1);
  std::vector<float> xs(N);
  typename std::vector<float>::iterator x;
  float val;
  for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h) {
    *x = val;
  }
  return xs;
}

template <typename T>
std::vector<T> cos(std::vector<T> angles_rad) {
  size_t N = angles_rad.size();
  std::vector<T> out(N);
  for (size_t i = 0; i < N; ++i) {
    out(i) = std::cos(angles_rad);
  }
  return out;
}

template <typename T>
std::vector<T> sin(std::vector<T> angles_rad) {
  size_t N = angles_rad.size();
  std::vector<T> out(N);
  for (size_t i = 0; i < N; ++i) {
    out(i) = std::sin(angles_rad);
  }
  return out;
}

/*
In numerical analysis, Simpson’s 1/3 rule is a method for numerical
approximation of definite integrals using parabolas to approximate each part of
the curve.
https://en.wikipedia.org/wiki/Simpson%27s_rule
*/

std::vector<float> cumparab(const std::vector<float>& x,
                            const std::vector<float>& fx) {
  // Note : In this Simpson's rule, n must be EVEN.
  auto n = x.size();
  assert(n > 0);
  assert(fx.size() == n);

  float h = (x[n] - x[0]) / n;

  std::vector<float> res(n);
  for (size_t i = 0; i <= n; i++) {
    if (i == 0 || i == n)
      res[i] = fx[i];
    else if (i % 2 != 0)
      res[i] = res[i - 1] + (4 * fx[i]);
    else
      res[i] = res[i - 1] + (2 * fx[i]);

    res[i] = res[i] * (h / 3);
  }

  return res;
}
template <class T>
T clamp(const T& v, const T& lo, const T& hi) {
  assert(!(hi < lo));
  return (v < lo) ? lo : (hi < v) ? hi : v;
}

double clampD(const double& v, const double& lo, const double& hi) {
  assert(!(hi < lo));
  return (v < lo) ? lo : (hi < v) ? hi : v;
}

}  // namespace utils