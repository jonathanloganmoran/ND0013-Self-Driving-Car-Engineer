/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: September 20, 2020
 *      Author: Munir Jojo-Verge
 **********************************************/

/**
 * @file: spiral_base.cpp
 **/
#include "spiral_base.h"

#include <cmath>
#include <limits>

SpiralBase::SpiralBase(const std::uint32_t order)
    : p_params_(order + 1, 0.0),
      sg_(0.0),
      error_(std::numeric_limits<double>::infinity()) {}

void SpiralBase::SetSpiralConfig(const SpiralConfig& spiral_config) {
  spiral_config_ = spiral_config;
}

// set params
void SpiralBase::set_start_point(const PathPoint& start) {
  start_point_ = &start;
}
void SpiralBase::set_end_point(const PathPoint& end) { end_point_ = &end; }

// output params
const PathPoint& SpiralBase::start_point() const { return *start_point_; }

const PathPoint& SpiralBase::end_point() const { return *end_point_; }

double SpiralBase::sg() const { return sg_; }

double SpiralBase::error() const { return error_; }

const std::vector<double>& SpiralBase::p_params() const { return p_params_; }
const SpiralConfig& SpiralBase::spiral_config() const { return spiral_config_; }

void SpiralBase::set_sg(const double sg) { sg_ = sg; }

void SpiralBase::set_error(const double error) { error_ = error; }

bool SpiralBase::ResultSanityCheck() const {
  for (const auto& p : p_params_) {
    if (std::isnan(p)) {
      return false;
    }
  }
  return (sg_ > 0);
}
