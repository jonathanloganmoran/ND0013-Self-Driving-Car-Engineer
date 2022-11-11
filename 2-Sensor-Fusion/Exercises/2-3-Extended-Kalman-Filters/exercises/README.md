# Exercises Extended Kalman Filter

Here are the instructions for the EKF exercises.

## 1_predict_update.py

 In this exercise, you will implement the predict and update functions of a linear Kalman filter in Python. 

### Your Task
Please implement the `predict()` and `update()` functions in the `Filter` class using the prepared `F()`, `Q()`, and `H()` functions. For matrix manipulation, I would like you to use the `numpy.matrix` format. Both functions return state and covariance.

### Desired Result

After successful completion, the console output should show that the state x and covariance P get updated in every iteration. The position estimation at time 100 should be close to the true position 100, and the velocity estimation should be close to the true velocity 1. The entries of P should decrease if the filter is stabilizing.

## 2_filter.py

In this exercise, you will implement the system matrix F, process noise covariance Q and the measurement matrix H for a linear Kalman filter.

### Your Task

 Please implement and return the system matrix `F()`, process noise covariance `Q()` and the measurement matrix `H()` in the `Filter` class for a constant velocity motion model. We use a 4D state vector with 2D position and 2D velocity in this exercise and a 2D lidar measurement. Please use the prepared attributes `dt` and `q` in the Filter class.

### Desired Result

Running the script should show an animation where the green Kalman filter results approximate the gray ground truth.