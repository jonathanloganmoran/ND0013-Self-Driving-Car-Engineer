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

### Here we use the `numpy.matrix` class
# Note that this class is being deprecated and its use in this programme will be
# switched to an `numpy.ndarray` implmenetation soon.
# See: https://numpy.org/devdocs/reference/generated/numpy.matrix.html#numpy.matrix
import numpy as np
from typing import Tuple


class Filter:
    '''The Kalman filter class.

    Implements the 1-D Kalman filter using State space form.
    
    Here we assume the process noise and measurement error to be modelled by
    Gaussian distributions with known covariance s.t. they remain stationary
    over time. This allows us to construct the filter as a MSE minimisation
    problem.

    :param dim_state: the number of dimensions of the process model `P`.
    :param u: the external motion model.
    :param F: the state transition matrix in the linear case
        i.e., system matrix.
    :param Q: the process noise covariance matrix, sampled at random from
        a Gaussian normal distribution.
    :param H: the measurement function relating the measurement and the motion
        of the object from one time-step to the next.
    '''

    def __init__(self):
        """Initialises the Kalman filter object with given dimensionality."""

        # The number of dimensions of the process model
        self.dim_state = 2
        # The external motion model (not used for this problem)
        self.u = np.array([[0., 0.]])
        # Instantiate the state transition matrix (i.e., system matrix)
        self.F = self.F()
        # Instantiate the process noise covariance matrix
        self.Q = self.Q()
        # Instantiate the measurement function
        self.H = self.H()


    def F(self
    ) -> np.ndarray:
        """Implements the state transition matrix.

        We refer to `F` as the system matrix, i.e., a linear matrix
        function governing the object movement model over time.

        :returns: F, the linear system matrix.
        """

        return np.array([[1, 1],
                         [0, 1]])

    def Q(self
    ) -> np.ndarray:
        """Implements the process noise covariance matrix.

        We refer to `Q` as the process noise covariance matrix, i.e.,
        the covariance of the process noise modelled after the stochastic
        variable $\nu$ from a Gaussian distribution with zero cross-correlation
        to the measurement noise.

        The larger the values of `Q`, the higher the expectation of noise.
        In e.g., an emergency braking system of a vehicle, `Q` might have
        relatively large values as the object motion model produces estimates
        with high uncertainty due to e.g., unexpected braking or acceleration.

        :returns: Q, the process noise covariance matrix.
        """

        return np.array([[0, 0],
                         [0, 0]])
        
    def H(self
    ) -> np.ndarray:
        """Implements the measurement function.

        We refer to `H` as a deterministic function relating the measurement
        of an object from time-step $t_{k-1}$ to $t_{k}$. The matrix `H` is
        the noiseless connection between the state vector and the measurement
        vector and is assumed to be stationary over time.

        :returns: H, the measurement matrix.
        """

        return np.array([[1, 0]])
    
    def predict(self, 
            x: np.ndarray, P: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
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

        ### Project the state estimate into the next time-step
        # Here we update the motion from $t_{k}$ to $t_{k+1}$
        x = self.F @ x + self.u
        ### Project the covariance matrix into the next time-step
        # Here the covariance process noise matrix `Q` accounts for uncertainty
        # in object motion model due to e.g., unexpected braking / acceleration.
        P = np.matmul(self.F @ P, self.F.T) + self.Q
        return x, P

    def update(self, 
            x: np.ndarray, P: np.ndarray, z: np.ndarray, R: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
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

        ### Compute the measurement residual update step (i.e., innovation step)
        # Here we compare the new measurement `z` with the prev. state estimate
        # transformed to the measurement space by matrix `H`
        gamma = z - self.H @ x
        ### Compute the covariance of the residual update
        # Here we transform the estimation error from covariance matrix `P` to
        # measurement space given by $H^{\top}H$ then add measurement noise `R`
        S = np.matmul(self.H @ P, self.H.T) + R
        ### Compute the Kalman gain
        # Here we weight the predicted state in comparison to the measurement
        K = np.matmul(P @ self.H.T, np.linalg.inv(S))
        ### Update the state estimate w.r.t. the weighted measurement
        # Here we give greater weight to either the measurement or the prev.
        # estimate using the Kalman gain `K`, i.e., the larger `K` the greater
        # the weight given to the residual measurement `gamma`
        x = x + K @ gamma
        ### Update the covariance matrix w.r.t. the weighted measurement
        # Here the identity $ P_{K} \times P_{k}^{-1} = I $ is used
        P = (np.identity(n=self.dim_state) - np.matmul(K, self.H)) @ P
        return x, P     
        
        
def run_filter():
    """Performs Kalman filtering over the measurement data."""

    ### Fix the seed s.t. random values are predictable
    np.random.seed(10)
    ### Instantiate the Kalman filter object
    KF = Filter()
    ### Initialise the tracking state vector and covariance matrix
    x = np.array([[0],
                  [0]])
    P = np.array([[5**2, 0],
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
        z = np.array([[i + np.random.normal(0, sigma_z)]])
        # Initialise the measurement covariance matrix
        R = np.array([[sigma_z**2]])
        print('z =', z)
        ### Perform the update step
        # Update the state vector and covariance matrix with the measurement
        x, P = KF.update(x, P, z, R)
        print('x+ =', x)
        print('P+ =', P)

        
if __name__ == '__main__':
    ### Run the main loop
    run_filter()