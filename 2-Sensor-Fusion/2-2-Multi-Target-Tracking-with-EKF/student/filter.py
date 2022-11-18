# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Define the `Filter` class and its core functionality,
#                        i.e., the measurement state transition function, the
#                        covariance matrices, predict and update steps.
#
# You should have received a copy of the Udacity license with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
#
# NOTE: The current version of this programme relies on Numpy to perform data 
#       manipulation, however, a platform-specific implementation, e.g.,
#       TensorFlow `tf.Tensor` data ops, is recommended.
# ------------------------------------------------------------------------------

### General package imports
import numpy as np
import os
import sys

### Add project directory to PYTHONPATH to enable relative imports
# Alternatively, use the `pip install ..` script with setuptools
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(
    os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__)))
)
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

### Tracking package imports
# Import the Kalman filter parameters for `dt` and `q`
import misc.params as params


class Filter:
    '''The Kalman filter class.

    Implements the Kalman filter in 3-D using State space form.

    Here we assume the process noise and measurement error are modelled by
    Gaussian distributions. The process noise is assumed to have zero-mean and
    depends on both the elapsed time-step $\Delta{t}$ and the uncertainty of the
    object motion w.r.t. acceleration as modelled by covariance matrix $Q$.

    The discretisation of $Q$ is such that noise through acceleration is assumed
    to be equal in both $x$ and $y$, i.e., $ \nu{x} = \nu{y} $.

    This implementation assumes a constant velocity model with height estimation,
    i.e., matrix `F` and `Q` will be 6D matrices.

    :param dim_state: the dimensions of the process model `P`.
    :param dt: the discrete time-step, i.e., $\Delta{t}$, which is assumed to
        be fixed in this implementation.
    :param q: the design parameter of the covariance process noise matrix $Q$,
        which is selected w.r.t. the expected maximum change in velocity.
    :param F: the state transition matrix in the 3-D case.
    :param Q: the discretised process noise covariance matrix.
    :param H: the measurement projection matrix of the linear LiDAR sensor.
    '''

    def __init__(self):
        """Initialises the Kalman filter object with attributes."""

        # Process noise dimensionality
        self.dim_state = params.dim_state
        # Discrete time interval (fixed)
        self.dt = params.dt
        # Process noise covariance design parameter
        self.q = params.q

    def F(self
    ) -> np.ndarray:
        """Implements the state transition function as a system matrix `F`.

        Here we assume a linear motion model with constant velocity in 3-D,
        i.e., $F$ is a 6-D state matrix of position and velocity estimated
        in $x$, $y$, and $z$.

        :returns: F, the state transition matrix.
        """
        
        return np.array([[1., 0., 0., self.dt, 0., 0.],
                         [0., 1., 0., 0., self.dt, 0.],
                         [0., 0., 1., 0., 0., self.dt],
                         [0., 0., 0., 1., 0., 0.],
                         [0., 0., 0., 0., 1., 0.],
                         [0., 0., 0., 0., 0., 1.]])
        

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############

        return 0
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        pass
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############

        return 0
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        return 0
        
        ############
        # END student code
        ############ 