/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/*
 * @file: spiral_equations.h
 */

#pragma once

#include <array>

class SpiralEquations {
 public:
  /**
   * @brief: convert p parameters to a parameters for cubic spiral
   * @params: [in] sg - the final length of spiral path from start to end point
   *            [in] p  - vector of params: p0, p1, p2, p3
   *                      p0 = kappa at s = 0
   *                      p1 = kappa at s = sg / 3
   *                      p2 = kappa at s = 2 * sg / 3
   *                      p3 = kappa at s = sg
   * @return: [out] a - parameter vec of cubic kappa spiral
   **/
  static std::array<double, 4> p_to_k3(const double sg,
                                       const std::array<double, 4>& p);

  /**
   * @brief: convert p parameters to a parameters for quintic spiral
   * @params: [in] sg - the final length of spiral path from start to end point
   *            [in] p  - vector of params: p0, p1, p2, p3, p4, p5
   *
   *                      p0 = kappa  at s = 0
   *                      p1 = dkappa at s = 0
   *                      p2 = ddkappa at s = 0
   *                      p3 = kappa at s = sg / 3
   *                      p4 = kappa at s = 2 * sg / 3
   *
   * @return: [out] a - parameter vec of quintic kappa spiral
   **/
  static std::array<double, 6> p_to_k5(const double sg,
                                       const std::array<double, 6>& p);

  /* ------------------------  kappa, theta, dkappa functions
   * --------------------- */

  // set 1: kappa, theta, dkappa with a vector (normal vector) as parameter
  // input
  /**
   * @brief : cubic kappa function with regular parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - cubic polynomial params for cubic kappa spiral
   * @return: kappa value with respect to s, kappa(s) given params vec = a
   **/
  static double kappa_func_k3_a(const double s, const std::array<double, 4>& a);

  /**
   * @brief : cubic theta function with regular parameter
   *            theta value is the integration of kappa value
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - cubic polynomial params for cubic kappa spiral
   * @return: theta value with respect to s, theta(s) given params vec = a
   **/
  static double theta_func_k3_a(const double s, const std::array<double, 4>& a);

  /**
   * @brief : derivative of cubic kappa function (dkappa) with regular parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - cubic polynomial params for cubic kappa spiral
   * @return: dkappa value with respect to s, dkappa(s) given params vec = a
   **/
  static double dkappa_func_k3_a(const double s,
                                 const std::array<double, 4>& a);

  /**
   * @brief : quintic kappa function with regular parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - quintic polynomial params for quintic kappa spiral
   * @return: kappa value with respect to s, kappa(s) given params vec = a
   **/
  static double kappa_func_k5_a(const double s, const std::array<double, 6>& a);

  /**
   * @brief : quintic theta function with regular parameter
   *            theta value is the integration of kappa value
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - quintic polynomial params for quintic kappa spiral
   * @return: theta value with respect to s, theta(s) given params vec = a
   **/
  static double theta_func_k5_a(const double s, const std::array<double, 6>& a);

  /**
   * @brief : derivative of quintic kappa function (dkappa) with regular
   *parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] a - quintic polynomial params for quintic kappa spiral
   * @return: dkappa value with respect to s, dkappa(s) given params vec = a
   **/
  static double dkappa_func_k5_a(const double s,
                                 const std::array<double, 6>& a);

  // set 2 - kappa, theta dkappa funcs with p parameter

  /**
   * @brief : cubic theta function with p parameter
   *            theta value is the integration of kappa value
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - p params for cubic kappa spiral
   * @return: theta value with respect to s, theta(s) given params vec = a
   **/
  static double kappa_func_k3(const double s, const double sg,
                              const std::array<double, 4>& p);

  /**
   * @brief : cubic theta function with p parameter
   *            theta value is the integration of kappa value
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - p params for cubic kappa spiral
   * @return: theta value with respect to s, theta(s) given p
   **/
  static double theta_func_k3(const double s, const double sg,
                              const std::array<double, 4>& p);

  /**
   * @brief : derivative of cubic kappa function (dkappa) with p parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - p params for cubic kappa spiral
   * @return: dkappa value with respect to s, dkappa(s) given p
   **/
  static double dkappa_func_k3(const double s, const double sg,
                               const std::array<double, 4>& p);

  /**
   * @brief : quintic kappa function with p parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - quintic polynomial params for quintic kappa spiral
   * @return: kappa value with respect to s, kappa(s) given p
   **/
  static double kappa_func_k5(const double s, const double sg,
                              const std::array<double, 6>& p);

  /**
   * @brief : quintic theta function with p parameter
   *            theta value is the integration of kappa value
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - quintic polynomial params for quintic kappa spiral
   * @return: theta value with respect to s, theta(s) given p
   **/
  static double theta_func_k5(const double s, const double sg,
                              const std::array<double, 6>& p);

  /**
   * @brief : derivative of quintic kappa function (dkappa) with regular
   *parameter
   * @params: [in] s - distance from start to current point on the path
   *            [in] p - quintic polynomial params for quintic kappa spiral
   * @return: dkappa value with respect to s, dkappa(s) given p params
   **/
  static double dkappa_func_k5(const double s, const double sg,
                               const std::array<double, 6>& p);

  /*** ------------------------- Partial Derivatives
   * -------------------------------------*/
  // Partial deriavatives of theta with respect to p1, p2, sg (p3, p4 sg for
  // quintic version)

  /**
   * @brief: calculate partial derivative given sg (final curve length) at s
   *location;
   * @params: [in] s  - s location with respect to path's own SL coordinate
   *            [in] sg - final length of path
   *            [in] p  - p params
   * @return partial theta / partial p1, p2 or p3
   **/

  // ----------------------------------- Cubic Version
  // ----------------------------------

  /**
   * @brief: partial derivative theta with respect to p1
   **/
  static double partial_theta_p1_k3(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to p2
   **/
  static double partial_theta_p2_k3(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to sg
   **/
  static double partial_theta_sg_k3(const double s, const double sg,
                                    const std::array<double, 4>& p);

  //  ---------------------------------- Quintic Version
  //  ---------------------------------

  /**
   * @brief: partial derivative theta with respect to p3
   **/
  static double partial_theta_p3_k5(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to p4
   **/
  static double partial_theta_p4_k5(const double s, const double sg);

  /**
   * @brief: partial derivative theta with respect to sg
   **/
  static double partial_theta_sg_k5(const double s, const double sg,
                                    const std::array<double, 6>& p);

 private:
  SpiralEquations() = default;
};
