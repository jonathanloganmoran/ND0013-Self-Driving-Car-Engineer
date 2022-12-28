/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/
/**
 * @file integral.h
 * @brief Functions to compute integral.
 */
#pragma once

#include <functional>
#include <vector>

double IntegrateBySimpson(const std::vector<double>& funv_vec, const double dx,
                          const std::size_t nsteps);

double IntegrateByTrapezoidal(const std::vector<double>& funv_vec,
                              const double dx, const std::size_t nsteps);
/**
 * @brief Compute the integral of a target single-variable function
 *        from a lower bound to an upper bound, by 5-th Gauss-Legendre method
 * Given a target function and integral lower and upper bound,
 * compute the integral approximation using 5th order Gauss-Legendre
 * integration.
 * The target function must be a smooth function.
 * Example:
 * target function: auto func = [](const double& x) {return x * x;};
 *                  double integral = gauss_legendre(func, -2, 3);
 * This gives you the approximated integral of function x^2 in bound [-2, 3]
 *
 * reference: https://en.wikipedia.org/wiki/Gaussian_quadrature
 *            http://www.mymathlib.com/quadrature/gauss_legendre.html
 *
 * @param func The target single-variable function
 * @param lower_bound The lower bound of the integral
 * @param upper_bound The upper bound of the integral
 * @return The integral result
 */
double IntegrateByGaussLegendre(const std::function<double(double)>& func,
                                const double lower_bound,
                                const double upper_bound);
