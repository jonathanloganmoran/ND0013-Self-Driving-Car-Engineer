/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file  : cubic_spiral.cpp
 **/

#include "cubic_spiral.h"

#include <algorithm>

#include <glog/logging.h>
#include "Eigen/Core"
#include "Eigen/LU"

#include "integral.h"
#include "spiral_equations.h"
#include "structs.h"

CubicSpiral::CubicSpiral() : SpiralBase(3) {
  // generate an order 3 cubic spiral path with four parameters
}

bool CubicSpiral::GenerateSpiral(const PathPoint& start, const PathPoint& end) {
  set_start_point(start);
  set_end_point(end);

  // starting p[oint
  double x_s = start_point().x;
  double y_s = start_point().y;
  double theta_s = std::fmod(start_point().theta, s_two_pi_);

  if (theta_s < 0) {
    theta_s += s_two_pi_;
  }

  // end point
  double x_t = end_point().x - x_s;
  double y_t = end_point().y - y_s;

  // with position and rotation transformation
  double x_g = std::cos(theta_s) * x_t + std::sin(theta_s) * y_t;
  double y_g = -std::sin(theta_s) * x_t + std::cos(theta_s) * y_t;
  double theta_g = std::fmod(end_point().theta, s_two_pi_);
  theta_g -= theta_s;

  while (theta_g < -M_PI) {
    theta_g += s_two_pi_;
  }

  while (theta_g > +M_PI) {
    theta_g -= s_two_pi_;
  }
  std::array<double, 4> p_shoot;
  double sg =
      (theta_g * theta_g / 5.0 + 1.0) * std::sqrt(x_g * x_g + y_g * y_g);
  p_shoot[0] = start_point().kappa;
  p_shoot[1] = 0.0;
  p_shoot[2] = 0.0;
  p_shoot[3] = end_point().kappa;

  // intermediate params
  Eigen::Matrix<double, 3, 1> q_g;
  q_g << x_g, y_g, theta_g;            // goal, x(p, sg), y(p, sg), theta(p, sg)
  Eigen::Matrix<double, 3, 3> jacobi;  // Jacobian matrix for newton method

  // simpson integrations func values in Jacobian
  // integration point initialization:
  double ds =
      sg / (spiral_config().simpson_size - 1);  // bandwith for integration
  // basic theta value vectors:
  std::vector<double> theta(spiral_config().simpson_size, 0.0);
  std::vector<double> cos_theta(spiral_config().simpson_size, 0.0);
  std::vector<double> sin_theta(spiral_config().simpson_size, 0.0);
  // partial derivatives vectors for Jacobian
  std::vector<double> ptp_p1(spiral_config().simpson_size, 0.0);
  std::vector<double> ptp_p2(spiral_config().simpson_size, 0.0);
  std::vector<double> ptp_sg(spiral_config().simpson_size, 0.0);
  std::vector<double> sin_ptp_p1(spiral_config().simpson_size, 0.0);
  std::vector<double> sin_ptp_p2(spiral_config().simpson_size, 0.0);
  std::vector<double> sin_ptp_sg(spiral_config().simpson_size, 0.0);
  std::vector<double> cos_ptp_p1(spiral_config().simpson_size, 0.0);
  std::vector<double> cos_ptp_p2(spiral_config().simpson_size, 0.0);
  std::vector<double> cos_ptp_sg(spiral_config().simpson_size, 0.0);

  // newton iteration difference (col) vectors
  Eigen::Matrix<double, 3, 1> delta_q;  // goal difference
  Eigen::Matrix<double, 3, 1> delta_p;  // parameter difference
  Eigen::Matrix<double, 3, 1>
      q_guess;        // q with current paramter, delta_q = q_g - q_guess
  double diff = 0.0;  // absolute error for q iteration stop

  for (int32_t nt = 0; nt < spiral_config().newton_raphson_max_iter; ++nt) {
    // calculate parameters for simpson integration
    double s = 0.0;

    for (int32_t i = 0; i < spiral_config().simpson_size; ++i) {
      theta[i] = SpiralEquations::theta_func_k3(s, sg, p_shoot);

      cos_theta[i] = std::cos(theta[i]);
      sin_theta[i] = std::sin(theta[i]);

      ptp_p1[i] = SpiralEquations::partial_theta_p1_k3(s, sg);
      ptp_p2[i] = SpiralEquations::partial_theta_p2_k3(s, sg);
      ptp_sg[i] = SpiralEquations::partial_theta_sg_k3(s, sg, p_shoot);

      sin_ptp_p1[i] = sin_theta[i] * ptp_p1[i];
      sin_ptp_p2[i] = sin_theta[i] * ptp_p2[i];
      sin_ptp_sg[i] = sin_theta[i] * ptp_sg[i];

      cos_ptp_p1[i] = cos_theta[i] * ptp_p1[i];
      cos_ptp_p2[i] = cos_theta[i] * ptp_p2[i];
      cos_ptp_sg[i] = cos_theta[i] * ptp_sg[i];
      s += ds;
    }

    // update Jacobian and delta q
    jacobi(0, 0) =
        -IntegrateBySimpson(sin_ptp_p1, ds, spiral_config().simpson_size);
    jacobi(0, 1) =
        -IntegrateBySimpson(sin_ptp_p2, ds, spiral_config().simpson_size);
    jacobi(0, 2) =
        cos_theta[spiral_config().simpson_size - 1] -
        IntegrateBySimpson(sin_ptp_sg, ds, spiral_config().simpson_size);

    jacobi(1, 0) =
        IntegrateBySimpson(cos_ptp_p1, ds, spiral_config().simpson_size);
    jacobi(1, 1) =
        IntegrateBySimpson(cos_ptp_p2, ds, spiral_config().simpson_size);
    jacobi(1, 2) =
        sin_theta[spiral_config().simpson_size - 1] +
        IntegrateBySimpson(cos_ptp_sg, ds, spiral_config().simpson_size);

    jacobi(2, 0) = ptp_p1[spiral_config().simpson_size - 1];
    jacobi(2, 1) = ptp_p2[spiral_config().simpson_size - 1];
    jacobi(2, 2) = ptp_sg[spiral_config().simpson_size - 1];

    q_guess(0) =
        IntegrateBySimpson(cos_theta, ds, spiral_config().simpson_size);
    q_guess(1) =
        IntegrateBySimpson(sin_theta, ds, spiral_config().simpson_size);
    q_guess(2) = theta[spiral_config().simpson_size - 1];

    delta_q = q_g - q_guess;

    diff =
        std::fabs(delta_q(0)) + std::fabs(delta_q(1)) + std::fabs(delta_q(2));

    if (diff < spiral_config().newton_raphson_tol) {
      break;
    }

    // solve by lu decomposition
    delta_p = jacobi.lu().solve(delta_q);
    // update p, sg, ds
    p_shoot[1] += delta_p(0);
    p_shoot[2] += delta_p(1);
    sg += delta_p(2);
    ds = sg / (spiral_config().simpson_size - 1);
  }

  PrependToPParams(p_shoot.begin(), p_shoot.end());
  set_sg(sg);
  set_error(diff);

  return diff < spiral_config().newton_raphson_tol && ResultSanityCheck();
}

bool CubicSpiral::GetSampledSpiral(const std::uint32_t n,
                                   std::vector<PathPoint>* path_points) const {
  CHECK_NOTNULL(path_points);

  // initialization
  if (n < 2 || error() > spiral_config().newton_raphson_tol) {
    return false;
  }

  path_points->resize(n);

  std::vector<PathPoint>& result = *path_points;
  const double ds = sg() / (n - 1);

  std::array<double, 4> p_value;
  std::copy_n(p_params().begin(), 4, p_value.begin());

  result[0].x = start_point().x;
  result[0].y = start_point().y;
  result[0].theta = start_point().theta;
  result[0].kappa = start_point().kappa;
  result[0].dkappa = SpiralEquations::dkappa_func_k3(0, sg(), p_value);

  // calculate path x, y using iterative trapezoidal method
  // initialization
  double s = ds;
  // calculate heading kappa along the path
  std::array<double, 4> a_params = SpiralEquations::p_to_k3(sg(), p_value);

  for (std::uint32_t i = 1; i < n; ++i) {
    result[i].s = s;
    result[i].theta =
        SpiralEquations::theta_func_k3_a(s, a_params) + result[0].theta;
    result[i].kappa = SpiralEquations::kappa_func_k3_a(s, a_params);
    result[i].dkappa = SpiralEquations::dkappa_func_k3_a(s, a_params);
    s += ds;
  }

  // integration x, y along the path
  double dx = 0;
  double dy = 0;

  for (std::uint32_t k = 1; k < n; ++k) {
    dx = (dx / k) * (k - 1) +
         (std::cos(std::fmod(result[k].theta, s_two_pi_)) +
          std::cos(std::fmod(result[k - 1].theta, s_two_pi_))) /
             (2 * k);
    dy = (dy / k) * (k - 1) +
         (std::sin(std::fmod(result[k].theta, s_two_pi_)) +
          std::sin(std::fmod(result[k - 1].theta, s_two_pi_))) /
             (2 * k);
    result[k].x = result[k].s * dx + result[0].x;
    result[k].y = result[k].s * dy + result[0].y;
  }

  return true;
}
