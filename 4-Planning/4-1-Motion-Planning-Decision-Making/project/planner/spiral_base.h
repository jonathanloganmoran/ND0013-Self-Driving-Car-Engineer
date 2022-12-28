/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file: spiral_base.h
 * @brief: spiral path base class
 **/

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "structs.h"

class SpiralBase {
 public:
  SpiralBase(const std::uint32_t order);
  virtual ~SpiralBase() = default;

  /**
   *   @brief: set configuration if desired (default setting was defined in
   *constructor)
   **/
  void SetSpiralConfig(const SpiralConfig& spiral_config);
  /**
   *   @brief : default process of calculating path without lookup table
   *   @return: errors of final state: fitted value vs true end point
   **/
  virtual bool GenerateSpiral(const PathPoint& start, const PathPoint& end) = 0;

  /**
   *   @brief: output methods
   **/
  const std::vector<double>& p_params() const;
  const SpiralConfig& spiral_config() const;
  const PathPoint& start_point() const;
  const PathPoint& end_point() const;
  void set_start_point(const PathPoint& start);
  void set_end_point(const PathPoint& end);
  double sg() const;
  double error() const;

  /**
   *   @brief : get path vector with sampling size n
   *   @return: sequence of sampling points
   **/
  virtual bool GetSampledSpiral(const std::uint32_t n,
                                std::vector<PathPoint>* path_points) const = 0;

 private:
  const PathPoint* start_point_;
  const PathPoint* end_point_;
  std::vector<double> p_params_;
  double sg_;
  double error_;
  SpiralConfig spiral_config_;

 protected:
  void set_sg(const double sg);
  void set_error(const double error);

  bool ResultSanityCheck() const;

  template <typename T>
  void PrependToPParams(T begin, T end) {
    std::copy(begin, end, p_params_.begin());
  }
  static constexpr double s_two_pi_ = 2 * M_PI;
};
