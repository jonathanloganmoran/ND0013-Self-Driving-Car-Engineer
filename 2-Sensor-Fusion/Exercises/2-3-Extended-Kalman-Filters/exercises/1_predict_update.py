# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Define the Kalman `Filter` class and its core
#                        functions, i.e., the `predict` and `update` steps.
#
# You should have received a copy of the Udacity license with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
#
# NOTE: The current version of this programme relies on Numpy to perform data 
#       manipulation, however, a platform-specific implementation, e.g.,
#       TensorFlow `tf.Tensor` data ops, is recommended.
# ------------------------------------------------------------------------------

import numpy as np


class Filter:
    '''The Kalman filter class.'''

    def __init__(self):
        """Initialises the Kalman filter object with given dimensionality."""

        # The number of dimensions of the process model
        self.dim_state = 2

    def F(self
) -> np.matrix:
        """Implements the state transition matrix.

        We refer to `F` as the system matrix, i.e., a linear matrix
        function governing the object movement model over time.

        :returns: F, the linear system matrix.
        """

        return np.matrix([[1, 1],
                          [0, 1]])

    def Q(self
) -> np.matrix:
        """Implements the process noise covariance matrix.

        We refer to `Q` as the process noise covariance matrix, i.e.,
        the covariance between the process noise modelled as a stochastic
        variable $\nu$.

        The larger the values of `Q`, the higher the expectation of noise.
        In e.g., an emergency braking system of a vehicle, `Q` might have
        relatively large values as the object motion model produces estimates
        with high uncertainty due to e.g., unexpected braking or acceleration.

        :returns: Q, the process noise covariance matrix.
        """

        return np.matrix([[0, 0],
                        [0, 0]])
        
    def H(self
) -> np.matrix:
        """Implements the measurement function.

        We refer to `H` as a deterministic function relating the measurement
        of an object from time-step $t_{k-1}$ to $t_{k}$.

        :returns: H, the measurement matrix.
        """

        return np.matrix([[1, 0]])
    
    def predict(self, 
        x: np.matrix, P: np.matrix
) -> Tuple[np.matrix, np.matrix]:
        """Implements the prediction step.

        The state estimate and covariance matrix are updated with respect to the
        next time-step.

        :param x: the observed state estimate from the previous time-step,
            i.e., $x^{+}$, the state estimate updated w.r.t. the weighted
            measurement observed at time $t_{k}$.
        :param P: the covariance matrix from the previous time-step,
            i.e., $P^{+}$, the covariance matrix updated w.r.t. the weighted
            measurement observed at time $t_{k}$.
        :returns: tuple, the predicted state estimate and covariance matrix.
        """

        ############
        # TODO: implement prediction step
        ############
        
        return x, P

    def update(self, 
            x, P, z, R
) -> Tuple[np.matrix, np.matrix]:
        """Implements the update step.

        Also referred to as the 'correction' step, here the state estimate and
        the covariance matrix are updated w.r.t. the measurement observed at
        time $t_{k}$.

        :param x: the state estimate obtained from the previous time-step,
            i.e., $x^{-}$, the predicted state estimate.
        :param P: the covariance matrix from the previous time-step,
            i.e., $P^{-}$, the predicted process noise covariance matrix.
        :param z: the estimate obtained from the previous time-step,
            i.e., the weighted predicted position and velocity of the object.
        :param R: the measurement noise covariance matrix,
            i.e., the model of expected error in the measurement observations.
        :returns: tuple, the updated state estimate and covariance matrix.
        """

        ############
        # TODO: implement update step
        ############
        
        return x, P     
        
        
def run_filter():
    """Performs Kalman filtering over the measurement data."""

    ### Fix the seed s.t. random values are predictable
    np.random.seed(10)
    ### Instantiate the Kalman filter object
    KF = Filter()
    ### Initialise the tracking state vector and covariance matrix
    x = np.matrix([[0],
                   [0]])
    P = np.matrix([[5**2, 0],
                   [0, 5**2]])
    ### Loop over the measurements and call predict and update
    for i in range(1,101):        
        print('------------------------------')
        print('processing measurement #' + str(i))
        
        ### Perform the prediction step
        x, P = KF.predict(x, P) # predict to next timestep
        print('x- =', x)
        print('P- =', P)
        ### Obtain the measurement model
        # Model the measurement noise
        sigma_z = 1
        # Initialise the noise at random from a Guassian normal distribution
        z = np.matrix([[i + np.random.normal(0, sigma_z)]])
        # Initialise the measurement covariance matrix
        R = np.matrix([[sigma_z**2]])
        print('z =', z)
        ### Perform the update step
        # Update the state vector and covariance matrix with the measurement
        x, P = KF.update(x, P, z, R)
        print('x+ =', x)
        print('P+ =', P)

        
if __name__ == '__main__':
    ### Run the main loop
    run_filter()