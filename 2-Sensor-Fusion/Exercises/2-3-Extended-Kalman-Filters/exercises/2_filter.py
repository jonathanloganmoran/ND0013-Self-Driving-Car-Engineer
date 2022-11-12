# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Define the 2-D Kalman `Filter` class and its core
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

import matplotlib
### Change Matplotlib backend for compatibility
# Using 'wxagg' backend so that figure maximizing works on Mac as well
# matplotlib.use('wxagg')
# Using 'agg' backend so that plotting works on Ubuntu 10.04.6 LTS
# Note that 'agg' is a non-GUI backend, so only figure saving will work
#matplotlib.use('agg')
import matplotlib.pyplot as plt
### Here we use the `numpy.matrix` class
# Note that this class is being deprecated and its use in this programme will be
# switched to an `numpy.ndarray` implmenetation soon.
# See: https://numpy.org/devdocs/reference/generated/numpy.matrix.html#numpy.matrix
import numpy as np

class Filter:
    '''The Kalman filter class.

    Implements the 2-D Kalman filter using State space form.

    Here we assume the process noise and measurement error are modelled by
    Gaussian distributions. The process noise is assumed to have zero-mean and
    depends on both the elapsed time-step $\Delta{t}$ and the uncertainty of the
    object motion w.r.t. acceleration as modelled by covariance matrix $Q$.

    The discretisation of $Q$ is such that noise through acceleration is assumed
    to be equal in both $x$ and $y$, i.e., $ \nu{x} = \nu{y} $.

    :param dim_state: the dimensions of the process model `P`.
    :param dt: the discrete time-step, i.e., $\Delta{t}$, which is no longer
        assumed to be constant as in the 1-D case.
    :param q: the design parameter of the covariance process noise matrix $Q$,
        which is selected w.r.t. the expected maximum change in velocity.
    '''

    def __init__(self):
        """Initialises the Kalman filter object with given dimensionality."""
        
        # The number of dimensions of the process model
        self.dim_state = 4
        # The external motion model (not used for this problem)
        self.u = np.matrix([[0., 0.]])
        # The discrete time-step measured in seconds (s)
        self.dt = 0.1
        # The design parameter of the covariance process noise
        self.q = 0.1

    def F(self
    ) -> np.ndarray:
        """Implements the state transition matrix.

        Here we assume a linear motion model with constant velocity in 2-D,
        i.e., $F$ is a 4-D state matrix of position and velocity estimated
        in both $x$ and $y$.

        :returns: F, the state transition matrix.
        """
        
        return np.matrix([[1., 0., self.dt, 0.],
                          [0., 1., 0., self.dt],
                          [0., 0., 1., 0.],
                          [0., 0., 0., 1.]])

    def Q(self
    ) -> np.ndarray:
        """Implements the process noise covariance matrix.

        We refer to `Q` as the process noise covariance matrix, i.e.,
        the covariance of the process noise modelled after the variable
        $\nu$ from a Gaussian distribution with zero-cross correlation to
        the measurement noise.

        The discretisation of $Q$ is such that noise through acceleration
        is assumed to be equal in both $x$ and $y$, i.e.,
            $ \nu_{x} = \nu_y} $.

        The $Q$ matrix depends on both the time-step $\Delta{t}$ and a process
        noise covariance design parameter `q`, which is selected w.r.t. the
        expected maximum change in velocity. For highly-dynamic manoeuvres,
        a higher parameter value, e.g., $q = 8 m/s^2$ is sufficient for
        emergency braking systems, whereas smaller values of `q`, e.g.,
        $q = 3 m/s^2$, are sufficient for normal driving conditions such as the
        highway driving scenario.

        :returns Q: the process noise covariance matrix.
        """

        return np.matrix([
            [self.dt**3 * self.q / 3., 0., self.dt**2 * self.q / 2., 0.],
            [0., self.dt**3 * self.q / 3., 0., self.dt**2 / 2.]
            [self.dt**2 * self.q / 2., 0., self.dt * self.q, 0.],
            [0., self.dt**2 * self.q / 2., 0., self.dt * self.q]])
    
    def H(self
    ) -> np.ndarray:
        """Implements the measurement function.

        We refer to `H` as the projection matrix from the 4-D state space of the
        object to the 2-D measurement space of the sensor, in this case, the
        linear LiDAR sensor.

        Here we discard the velocity information, $(v_{x}, v_{y})$, from the
        predicted state vector $ x = (p_{x}, p_{y}, v_{x}, v_{y})^{\top} $.
        This allows us to correctly project the 2-D components of position into
        the measurement space.

        :returns: H, the measurement model of the linear LiDAR sensor in 2-D.
        """

        return np.matrix([[1., 0., 0., 0.],
                          [0., 1., 0., 0.]])
    
    def predict(self,
            x: np.ndarray, P: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Implements the prediction step.

        The state estimate and covariance matrix are updated with respect to the
        next time-step.
        
        :param x: the observed state estimate from the previous time-step,
            i.e., $x^{+}$, the state estimated w.r.t. the weighted
            measurement observed at time $t_{k}$.
        :param P: the covariance matrix from the previous time-step,
            i.e., $P^{+}$, the covariance matrix updated w.r.t. the
            weighted measurement observed at time $t_{k}$.
        :returns: tuple, the predicted state estimate and covariance matrix.
        """

        ### Project the state estimate into the next time-step
        # Here we update the motion from $t_{k}$ to $t_{k+1}$
        x = self.F() * x + self.u
        ### Project the covariance matrix into the next time-step
        # Here the covariance process noise matrix `Q` accounts for uncertainty
        # in object motion model due to e.g., unexpected braking / acceleration
        P = self.F() * P * self.F().transpose() + self.Q()
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
        # transformed to the measurement space by `H`
        gamma = z - self.H() * x
        ### Compute the covariance of the residual update
        # Here we transform the estimation error from covariance matrix `P` to
        # measurement space given by $H^{\top}H$ then add measurement noise `R`
        S = self.H() * P * self.H.transpose() + R
        ### Compute the Kalman gain
        # Here we weight the predicted state in comparison to the measurement
        K = P * self.H.transpose() * np.linalg.inv(S)
        ### Update the state estimate w.r.t. the weighted measurement
        # Here we give greater weight to either the measurement or the prev.
        # estimate using the Kalman gain `k`, i.e., the larger the `K` the greater
        # the weight given to the residual measurement `gamma`
        x = x + K * gamma
        ### Update the covariance matrix w.r.t. the weighted measurement
        # Here the identity $ P_{k} \times P_{k}^{-1} = I $ is used
        I = np.identity(self.dim_state)
        P = (I - K * self.H()) * P
        return x, P
        
        
def run_filter():
    """Performs Kalman filtering over the measurement data."""
    
    # Fix the seed s.t. random values are predictable
    np.random.seed(0)
    # Instantiate the Kalman filter object
    KF = Filter()
    # Initialise the Matplotlib figure
    fig, ax = plt.subplots()
    # Initialise the track state vector and covariance matrix
    x = np.matrix([[0],
                   [0],
                   [0],
                   [0]]
    )
    P = np.matrix([[0.1**2, 0, 0, 0],
                   [0, 0.1**2, 0, 0],
                   [0, 0, 2**2, 0],
                   [0, 0, 0, 2**2]]
    )
    # Loop over the measurements and call `predict` and `update`
    for i in range(1, 101):
        ### Perform the prediction step
        # Here we predict to the next time-step
        x, P = KF.predict(x, P)
        # Here we generate the ground-truth observation as a
        # non-linear motion model
        gt = np.matrix([[i * KF.dt], 
                        [0.1 * (i * KF.dt)**2]]
        )
        ### Obtain the measurement using a linear motion model
        # The measurement noise constant to account for discrepency
        # between linear and non-linear motion models
        sigma_z = 0.2
        # Here we generate a noisy measurement from a Guassian normal distribution
        z = np.matrix([[float(gt[0]) + np.random.normal(0, sigma_z)],
                       [float(gt[1]) + np.random.normal(0, sigma_z)]]
        )
        # Here we generate the measurement noise covariance matrix
        # which is the uncertainty in the position measurements received by the
        # LiDAR sensor modelled by the the standard deviations of the detections
        # in $x$ and $y$ along the diagonal of $R$
        # Note that the off-diagonal 0s here indicate that the noise in $x$ and $y$
        # are in fact uncorrelated, per the assumption made w.r.t. $\gamma$
        R = np.matrix([[sigma_z**2, 0],
                       [0, sigma_z**2]]
        )
        ### Perform the update step
        # Update the state and covariance matrix with the measurement
        x, P = KF.update(x, P, z, R)
        ### Visualise the predicted and ground-truth tracks
        # Plot the tracks
        ax.scatter(float(x[0]), float(x[1]), color='green', s=40, marker='x', label='track')
        ax.scatter(float(z[0]), float(z[1]), color='blue', marker='.', label='measurement')
        ax.scatter(float(gt[0]), float(gt[1]), color='gray', s=40, marker='+', label='ground truth')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_xlim(0, 10)
        ax.set_ylim(0, 10)
        # Maximise the figure window   
        mng = plt.get_current_fig_manager()
        mng.frame.Maximize(True)
        # Remove any repeated labels from the plot
        handles, labels = ax.get_legend_handles_labels()
        handle_list, label_list = [], []
        for handle, label in zip(handles, labels):
            if label not in label_list:
                handle_list.append(handle)
                label_list.append(label)
        ax.legend(handle_list, label_list, loc='center left', shadow=True, fontsize='x-large', bbox_to_anchor=(0.8, 0.5))
        plt.pause(0.01)
    plt.show()


if __name__ == '__main__':
    ### Run the main loop
    run_filter()