# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Define the `Measurement`, `Track` and `Assocation`
#                        classes and their core functionalities, i.e., track
#                        state, measurement vector, covariance matrices and
#                        the Single Nearest Neighbour (SNN) association using
#                        a minimisation over Mahalanobis distances.                    
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
# Using 'agg' backend so that plotting works on Ubuntu 16.04.6 LTS
# Note that 'agg' is a non-GUI backend, so only figure saving will work
# matplotlib.use('agg')
# matplotlib.use('wxagg')
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from typing import List


class Measurement(object):
    '''The Measurement class.

    :param id: the unique measurement id.
    :param z: the measurement vector containing the position estimate.
    :param R: the measurement error covariance matrix.
    '''

    def __init__(self,
            id: int, x: np.ndarray, y: np.ndarray
    ):
        """Initialises a new Measurement instance.

        :param id: the unique measurement id.
        :param x: the x-coordinate of the measured position.
        :param y: the y-coordinate of the measured position.
        """

        ### Initialise the new measurement
        # Set the unique measurement id
        self.id = id
        # Generate the measurement vector
        self.z = np.array([
                    [x + np.random.normal(0, 2)],
                    [y + np.random.normal(0, 2)]
        ])
        # Initialise the measurement error covariance matrix
        self.R = np.array([
                    [2, 0],
                    [0, 2]
        ])


class Track(object):
    '''The Track class.

    :param id: the unique track id.
    :param x: the state vector containing the position estimate.
    :param P: the estimation error covariance matrix.
    '''

    def __init__(self, 
            id: int
    ):
        """Initialises a new Track instance.

        :param id: the unique track id.
        """

        ### Initialise a new track
        # Set the unique track id
        self.id = id
        # Generate a new state vector
        self.x = np.array([
                    [np.random.uniform(2, 8)],
                    [np.random.uniform(-3, 3)],
                    [0],
                    [0]
        ])
        # Initialise the estimation error covariance matrix
        self.P = np.array([
                    [2, 0, 0, 0],
                    [0, 3, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
        ])


class Association(object):
    '''The Association class.

    Implements the Single Nearest Neighbour (SNN) association and gating based
    on the Mahalanobis distance metric.

    :param association_matrix: the data association matrix used to assign
        uncertain measurements to known tracks.
    '''

    def __init__(self):
        """Initialises a new Association instance with matrix."""

        # Instantiate the association matrix
        self.association_matrix = np.array([])
        
    def associate(self,
            track_list: List[Track], meas_list: List[Measurement]
    ):
        """Implements the association algorithm.

        :param track_list: the list of known tracks that have not been assigned
            to a measurement.
        :param meas_list: the list of uncertain measurements that have not been
            assigned to a track.
        """

        # The number of tracks left to assign
        N = len(track_list)
        # The number of measurements left to assign
        M = len(meas_list)
        # Initialise the association matrix values
        self.association_matrix = np.inf * np.ones((N, M))
        for x_i, track in enumerate(track_list):
            for z_j, measurement in enumerate(meas_list):
                # Compute the Mahalanobis distance
                dist = self.calc_mhd(track, measurement)
                # Update the entry in the matrix with the distance value
                self.association_matrix[x_i, z_j] = dist

    def calc_mhd(self,
            track: Track, meas: Measurement
    ) -> float:
        """Implements the Mahalanobis distance calculation.

        :param track: the Track instance with known estimation error covariance.
        :param meas: the Measurement instance with uncertain position estimate
            and its measurement error covariance.
        :returns: the Mahalanobis distance measure between the given track
            and the measurement objects.
        """
        
        ### Compute the measurement matrix for the LiDAR sensor
        # Here we form a projection from 4D state space to 2D LiDAR measurement space
        _H = np.array([
                    [1., 0., 0., 0.],
                    [0., 1., 0., 0.]
        ])
        ### Compute the residual and its covariance for the LiDAR Sensor
        # Here we compute the residual
        gamma = meas.z - _H @ track.x
        # Here we compute the covariance of the residual
        _S = np.matmul(_H @ track.P, _H.T) + meas.R
        ### Compute the Mahalanobis distance
        dist = np.matmul(gamma.T @ np.linalg.inv(_S), gamma)
        return float(dist)


def run():
    """Tests the track / measurement association and visualises the results."""

    ### Initialise the association variables
    # Fix the seed s.t. random values are predictable
    np.random.seed(5)
    # Instantiate the data association instance
    association = Association()
    # Initialise the association lists
    track_list = []
    meas_list = []
    ### Create the Matplotlib figure instance
    # Initialise a new subplot
    fig, ax = plt.subplots(1, 1, figsize=(24, 20), constrained_layout=True)
    # Set the figure title
    txt1 = 'Track and Measurement Association: Mahalanobis Distance'
    fig.suptitle(txt1, fontsize=20)
    ### Simulate measurements and run the data association task
    for i in range(3):
        # Create a new track instance
        track = Track(i + 1)
        # Append the new track to the known tracks list
        track_list.append(track)
        # Plot the track
        ax.scatter(float(-track.x[1]), float(track.x[0]),
                        marker='x', s=95, color='red', label='Track'
        )
        # Set the track label
        ax.text(float(-track.x[1] + 0.05), float(track.x[0] + 0.05),
                        str(track.id), fontsize=16, color='red'
        )
        # Create a new measurement instance
        meas = Measurement(i + 1, float(track.x[0]), float(track.x[1]))
        # Append the new measurement to the uncertain measurements list
        meas_list.append(meas)
        # Plot the measurement
        ax.scatter(float(-meas.z[1]), float(meas.z[0]),
                        marker='o', s=95, color='green', label='Measurement'
        )
        # Set the measurement label
        ax.text(float(-meas.z[1] + 0.05), float(meas.z[0] + 0.05),
                        str(meas.id), fontsize=16, color='green'
        )
    ### Calculate the association matrix
    association.associate(track_list, meas_list)
    print('Association matrix:', association.association_matrix)
    ### Visualise the Mahalanobis distances
    for track in track_list:
        for meas in meas_list:
            # Get the track-measurement distance
            dist = association.association_matrix[track.id - 1, meas.id - 1]
            if dist < np.inf:
                # Plot the distance
                ax.plot([float(-track.x[1]), float(-meas.z[1])],
                        [float(track.x[0]), float(meas.z[0])],
                        color='gray', label='Distance'
                )
                # Set the distance label
                str_dist = "{:.2f}".format(dist)
                ax.text(
                    float((-track.x[1] - meas.z[1]) / 2), 
                    float((track.x[0] + meas.z[0]) / 2), 
                    str_dist, fontsize=14
                )
    # Maximise the figure window
    if matplotlib.rcParams['backend'] == 'wxagg':
        mng = plt.get_current_fig_manager()
        mng.frame.Maximize(True)
    ### Remove any repeated labels from the figure
    handles, labels = ax.get_legend_handles_labels()
    handle_list, label_list = [], []
    for handle, label in zip(handles, labels):
        if label not in label_list:
            handle_list.append(handle)
            label_list.append(label)
    ### Set the figure properties
    # Initialise the legend
    ax.legend(handle_list, label_list, loc='center left',
                    shadow=True, fontsize='large', bbox_to_anchor=(0.9, 0.05)
    )
    # Set the axes labels
    ax.set_xlabel('y [m]', fontsize=14)
    ax.set_ylabel('x [m]', fontsize=14)
    # Set the axes limits
    ax.set_xlim(-5, 5)
    ax.set_ylim(0, 10)
    # Correct the x-axis ticks so that the positive values are to the left
    ticks_x = ticker.FuncFormatter(
        lambda x, pos: '{0:g}'.format(-x) if x != 0 else '{0:g}'.format(x)
    )
    ax.xaxis.set_major_formatter(ticks_x)
    ### Show the resulting plot
    if matplotlib.rcParams['backend'] != 'agg':
        plt.show()


if __name__ == '__main__':
    ### Run the association programme
    run()