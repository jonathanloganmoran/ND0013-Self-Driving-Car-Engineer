# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Define the parameters for the tracking algorithm.             
#
# You should have received a copy of the Udacity license with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
#
# NOTE: The current version of this programme relies on Numpy to perform data 
#       manipulation, however, a platform-specific implementation, e.g.,
#       TensorFlow `tf.Tensor` data ops, is recommended.
# ------------------------------------------------------------------------------

### General parameters
# Process model dimension
dim_state = 6

### Step 1 : Kalman filter parameters
# Discrete time increment (fixed)
dt = 0.1
# Process noise design parameter for covariance `Q`
q = 3

### Step 2 : Track management parameters
# Track score threshold from 'tentative' to 'confirmed'
confirmed_threshold = 0.8
# Track score threshold to delete 'confirmed' tracks
delete_threshold = 0.6
# Track score threshold to delete 'initialized' tracks
delete_init_threshold = 0.17
# Number of frames in window for track score calculation
window = 6
# Covariance threshold in `px` or `py` to delete highly uncertain tracks
max_P = 3**2
# Initial values for the estimation error covariance `P` entries 
sigma_p44 = 50
sigma_p55 = 50
sigma_p66 = 5
# Sliding average parameter for dimension estimation
weight_dim = 0.1

### Step 3 : Association parameters
# Percentage of correct measurements assumed to be inside the gate
gating_threshold = 0.995

### Step 4 : Measurement parameters
# Measurement noise standard deviation for LiDAR sensor
sigma_lidar_x = 0.1
sigma_lidar_y = 0.1
sigma_lidar_z = 0.1
# Measurement noise standard deviation for camera sensor
sigma_cam_i = 5
sigma_cam_j = 5
