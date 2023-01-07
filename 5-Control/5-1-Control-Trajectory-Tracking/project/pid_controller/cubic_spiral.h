/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file  : cubic_spiral.h
 * @brief : path class includes the basic parameters for defining a path from
 *initial
 *            point to end point
 * @model description :
 *            x_p (s) = int_0^s cos( theta_p (s)) ds
 *            y_p (s) = int_0^s sin( theta_p (s)) ds
 *            theta_p (s) = a s + b s^2 / 2 + c s^3 / 3 + d s^4 / 4
 *            kappa_p (s) = a + b s + c s^2 + d s^3
 * @solver: Solve boundary shooting problem with newton raphson method
 *            (default) initialized step for newton: 8, tol = 10^-2, max_iter =
 *10
 **/

#pragma once
#include <vector>

#include <glog/logging.h>

#include "spiral_base.h"
#include "structs.h"

class CubicSpiral : public SpiralBase {
 public:
  CubicSpiral();
  ~CubicSpiral() = default;
  bool GenerateSpiral(const PathPoint& start, const PathPoint& end);
  bool GetSampledSpiral(const std::uint32_t n,
                        std::vector<PathPoint>* path_points) const override;
};