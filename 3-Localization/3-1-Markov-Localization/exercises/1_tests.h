/* ------------------------------------------------------------------------------
 * Lesson "3.1: Markov Localization"
 * Copyright (c) Aaron Brown, Tiffany Huang and Maximilian Muffert.
 *
 * Modified by: Jonathan L. Moran (jonathan.moran107@gmail.com)
 *
 * Purpose of this file: Header file for the Lesson 3.1 function tests.
 * ----------------------------------------------------------------------------
 */


#include "1_initialize_priors.h"
#include "2_determine_probabilities.h"
#include "4_coding_the_motion_model.h"
#include "6_get_pseudo_ranges.h"
#include "7_coding_the_observation_model.h"
#include "8_coding_the_full_filter.h"


// Exercise 3.1.1 : Initialize Priors Function
void test_initialize_priors();


// Exercise 3.1.2 : Determine Probabilities
void test_determine_probabilities();


// Exercise 3.1.3 : Motion Model Probability II
void test_motion_model_probability();


// Exercise 3.1.4 : Coding the Motion Model
void test_motion_model();


// Exercise 3.1.5 : Observation Model Probability
void test_observation_model_probability();


// Exercise 3.1.6 : Get Pseudo Ranges
void test_pseudo_range_estimator();


// Exercise 3.1.7 : Coding the Observation Model
void test_observation_model();


// Exercise 3.1.8 : Coding the Full Filter
void test_markov_filter();