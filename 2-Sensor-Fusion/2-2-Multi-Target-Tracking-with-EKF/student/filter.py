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
# Import the Measurement class for typing hints
from student.measurements import Measurement
# Import the Kalman filter parameters for `dt` and `q`
import misc.params as params
# Import Track class for typing hints
from student.trackmanagement import Track


class Filter(object):
    '''The Kalman filter class.

    Implements the Kalman filter in 3-D using state-space form.

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

    def Q(self
    ) -> np.ndarray:
        """Implements the process noise covariance matrix.

        We refer to `Q` as the process noise covariance matrix, i.e.,
        the covariance of the process noise modelled after the variable
        $\nu$ from a Gaussian distribution with zero-cross correlation to
        the measurement noise.

        The discretisation of $Q$ is such that noise through acceleration
        is assumed to be equal in both $x$, $y$ and $z$, i.e.,
            $ \nu_{x} = \nu_{y} = \nu_{z} $.

        The $Q$ matrix depends on both the time-step $\Delta{t}$ and a process
        noise covariance design parameter `q`, which is selected w.r.t. the
        expected maximum change in velocity. For highly-dynamic manoeuvres,
        a higher parameter value, e.g., $q = 8 m/s^2$ is sufficient for
        emergency braking systems, whereas smaller values of `q`, e.g.,
        $q = 3 m/s^2$, are sufficient for normal driving conditions such as the
        highway driving scenario.

        :returns Q: the process noise covariance matrix.
        """

        ### Compute the system matrix in the case that `dt` has changed
        _F = self.F()
        ### Discretising the continuous model
        # Assuming noise through acceleration is equal in x, y, and z
        _Q = np.diag([0., 0., 0., self.q, self.q, self.q])
        # The matrix exponential
        # Here the integral factor is evaluated from t=0 to t=dt
        _integral_factor = np.array([
            [self.dt / 3, 0., 0., self.dt / 2, 0., 0.],
            [0., self.dt / 3., 0., 0., self.dt / 2, 0.],
            [0., 0., self.dt / 3, 0., 0., self.dt / 2],
            [self.dt / 2, 0., 0., self.dt, 0., 0.],
            [0., self.dt / 2, 0., 0., self.dt, 0.],
            [0., 0., self.dt / 2, 0., 0., self.dt]])
        QT = _integral_factor * np.matmul(_F @ _Q, _F.T)
        return QT.T

    def predict(self,
            track: Track
    ):
        """Implements the prediction step.

        The state estimate $x^{+}$$ and covariance matrix $P^{+}$ of the given
        Track instance are updated with respect to the next time-step.
     
        :param track: the Track instance containing the state estimation
            and the covariance matrix from the previous time-step.
        """

        ### Compute the motion and covariance in the case that `dt` has changed
        _F = self.F()
        _Q = self.Q()
        ### Project the state estimate and covariance into the next time-step
        # Assuming a zero contribution due to an external motion model
        _x = self.F @ track.x
        # Adding uncertainty to the object motion model due to
        # e.g., unexpected braking / acceleration via covariance `Q` 
        _P = np.matmul(_F @ track.P, _F.T) + _Q
        # Update the track state and covariance
        track.set_x(_x)
        track.set_P(_P)

    def update(self, 
            track: Track,
            meas: Measurement
    ):
        """Implements the update step.

        Also referred to as the 'correction' step, here the state estimate and
        the covariance matrix are updated w.r.t. the measurement observed at
        time $t_{k}$.

        :param track: the Track instance containing the state estimate and the
            process noise covariance matrix updated from the prediction step.
        :param meas: the Measurement instance containing the measurement vector
            and the measurement noise covariance.
        """

        ### Compute the measurement residual update step (i.e., innovation step)
        _gamma = self.gamma(track, meas)
        ### Compute the covariance of the residual update
        # Getting the linearised measurement matrix (i.e., the Jacobian)
        _H = meas.sensor.get_H(track.x)
        _S = self.S(track, meas, _H)
        ### Compute the Kalman gain
        # Weighting the predicted state in comparison to the measurement
        _K = np.matmul(track.P @ _H.T, np.linalg.inv(_S))
        ### Update the state estimate w.r.t. the weighted measurement
        # Giving greater weight to either the measurement or the prev. estimate
        # using the Kalman gain `K`, i.e., the larger the `K` the greater
        # the weight given to the residual measurement `gamma`
        _x = track.x + _K @ _gamma
        # Setting the updated state estimate
        track.set_x(_x)
        ### Update the covariance matrix w.r.t. the weighted measurement
        # The identity $\mathrm{P}_{k} \times \mathrm{P}_{k}^{-1} = I$ is used.
        _I = np.identity(n=self.dim_state)
        _P = (_I - np.matmul(_K, _H)) @ track.P
        # Setting the updated measurement error covariance
        track.set_P(_P)
        ### Update the dimensions using exponential sliding average
        track.update_attributes(meas)
    
    def gamma(self,
            track: Track,
            meas: Measurement
    ) -> np.ndarray:
        """Helper function to compute and return the residual $\gamma$.

        The residual is the difference between the new measurement $\mathrm{z}$
        and the previous state estimate transformed to the measurement space.

        :param track: the Track instance containing the state estimate and the
            process noise covariance matrix updated from the prediction step.
        :param meas: the Measurement instance containing the measurement vector
            and the measurement noise covariance.
        :returns: gamma, the measurement residual update.
        """

        return meas.z - meas.sensor.get_hx(track.x)

    def S(self,
            track: Track,
            meas: Measurement,
            H: np.ndarray
    ) -> np.ndarray:
        """Helper function to compute and return the residual covariance $\mathrm{S}$.
        
        The estimation error covariance $\mathrm{P}$ is transformed to measurement
        space given by $\mathrm{H}\mathrm{H}^{\top}$. Then, the measurement noise
        covariance $\mathrm{R}$ is added.

        :param track: the Track instance containing the estimation error covariance.
        :param meas: the Measurement instance containing the measurement noise covariance.
        :param H: the Jacobian of the measurement model.
        :returns: S, the estimation error covariance of the residual.
        """
        
        return np.matmul(H @ track.P, H.T) + meas.R