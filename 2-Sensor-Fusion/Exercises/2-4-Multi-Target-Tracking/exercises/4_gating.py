# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Define the `Association`, `Track` and `Measurement`
#                        classes and their core functionalities, i.e., track
#                        state, measurement vector, covariance matrices and
#                        the Single Nearest Neighbour (SNN) association using
#                        a minimisation over Mahalanobis distances. Here we
#                        implement a 1-D validation gating algorithm.               
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
from scipy.stats.distributions import chi2
from typing import List, Tuple


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

    Implements the Single Nearest Neighbour (SNN) association and
    1-D validation gating based on the Mahalanobis distance metric.

    The SNN is a non-Bayesian approach assuming that each track generates
    at most one measurement and that each measurement originates from
    at most one track.

    The validation gate is a region of acceptance such that a percentage
    $100 * (1 - \alpha)$ of true measurements are rejected.

    :param association_matrix: the data association matrix used to assign
        uncertain measurements to known tracks.
    :param unassigned_tracks: the list of known tracks that have not been
        assigned to a measurement.
    :param unassigned_meas: the list of uncertain measurements that have not
        been assigned to a track.
    '''

    def __init__(self):
        """Initialises the Association instance and its attributes."""

        self.association_matrix = np.array([])
        self.unassigned_tracks = []
        self.unassigned_meas = []
        
    def associate(self, 
            track_list: List[Track], meas_list: List[Measurement]
    ):
        """Implements the assocation algorithm.

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
        return dist
    
    def gating(self,
            dist_mh, p_thresh=0.95, df_z=2
    ) -> bool:
        """Checks if the measurement is inside the gating region.

        The 1-D validation gate assumes a Gaussian measurement model
        with residual covariance $\mathrm{S}$. We define the area of the
        validation gate to be a hyper-ellipsoid which follows a Chi-square
        distribution, assuming that the Mahalanobis distance is a sum of
        squared standard normally-distributed random variables.

        The probability of a measurement `z` being inside the gating area is
        given w.r.t. a gating threshold $\gamma$ which can be obtained from
        the inverse Chi-square cumulative distribution for a significance level
        $\alpha$.

        :param dist_mh: the Mahalanbois distance.
        :param p_thresh: optional, the probability threshold, i.e., percentage
            of true measurements assumed to be within the gating area.
        :param df_z: optional, the degrees of freedom (DoF) of the measurement
            space of vector $z$.
        :returns: boolean, True if the measurement lies within the gate area.
        """ 

        ### Compute the inverse of the Chi-square cumulative distribution (cdf)
        # Using the percent point function (i.e., inverse of cdf)
        ppf = chi2.ppf(p=p_thresh, df=df_z)
        ### Check the Mahalanobis distance measure against the limit
        if dist_mh < ppf:
            return True
        else:
            return False
        
    def get_closest_track_and_meas(self
    ) -> Tuple[int, int]:
        """Return the indices of the closest track and measurement.

        Finds the closest pair and deletes their entries in the association matrix
        along with the corresponding ids in the `unassigned_tracks` and
        `unassigned_meas` lists.
        
        :returns: tuple, the closest unassociated track and measurement objects.
        """

        ### Find the closest track and measurement
        # Return NaN if no valid track / measurement pairs exist
        if np.min(self.association_matrix) == np.inf:
            return np.nan, np.nan
        # Get the indices of the entry with the lowest score
        idx_track, idx_measurement = np.unravel_index(
                indices=np.argmin(self.association_matrix, axis=None),
                shape=self.association_matrix.shape
        )
        ### Get the closest track and measurement objects
        track_closest = self.unassigned_tracks[idx_track]
        measurement_closest = self.unassigned_meas[idx_measurement]
        ### Remove the track and measurement objects from the lists
        self.unassigned_tracks.remove(track_closest)
        self.unassigned_meas.remove(measurement_closest)
        ### Remove the track-measurement pair from the association matrix
        _A = np.delete(self.association_matrix, idx_track, axis=0)
        self.association_matrix = np.delete(_A, idx_measurement, axis=1)
        return track_closest, measurement_closest
         
    
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
    fig, ax = plt.subplots(1, 1)
    ### Simulate measurements and run the data association task
    for i in range(3):
        # Create a new track instance
        track = Track(i + 1)
        # Append the new track to the known tracks list
        track_list.append(track)
        # Plot the track
        ax.scatter(float(-track.x[1]), float(track.x[0]),
                        marker='x', color='red', label='Track'
        )
        # Set the track label
        ax.text(float(-track.x[1]), float(track.x[0]),
                        str(track.id), color='red'
        )
        # Create a new measurement instance
        meas = Measurement(i + 1, float(track.x[0]), float(track.x[1]))
        # Append the new measurement to the uncertain measurements list
        meas_list.append(meas)
        # Plot the measurement
        ax.scatter(float(-meas.z[1]), float(meas.z[0]),
                        marker='o', color='green', label='Measurement'
        )
        # Set the measurement label
        ax.text(float(-meas.z[1]), float(meas.z[0]),
                        str(meas.id), color='green'
        )
    ### Calculate the association matrix with validation gating
    association.associate(track_list, meas_list)
    print('Association matrix:', association.association_matrix)
    print('unassigned_tracks list:', association.unassigned_tracks)
    print('unassigned_meas list:', association.unassigned_meas)     

    ### Visualise the Mahalanobis distances
    for track in track_list:
        for meas in meas_list:
            dist = association.association_matrix[track.id - 1, meas.id - 1]
            if dist < np.inf:
                # Plot the distance
                ax.plot(
                    [float(-track.x[1]), float(-meas.z[1])],
                    [float(track.x[0]), float(meas.z[0])],
                    color='gray'
                )
                # Set the distance label
                str_dist = "{:.2f}".format(dist)
                ax.text(
                    float((-track.x[1] - meas.z[1]) / 2),
                    float((track.x[0] + meas.z[0]) / 2),
                    str_dist
                )
    ### Update the associated tracks with the measurements
    matrix_orig = association.association_matrix
    while (
        association.association_matrix.shape[0] > 0
        and association.association_matrix.shape[1] > 0
    ):
        ### Get next closest track and measurement
        ind_track, ind_meas = association.get_closest_track_and_meas()
        if np.isnan(ind_track):
            print('---no more associations---')
            break
        ### Fetch the track, measurement and Mahalnobis distance
        track = track_list[ind_track]
        meas = meas_list[ind_meas]
        dist = matrix_orig[ind_track, ind_meas]
        ### Plot the track and measurement association
        ax.plot(
            [float(-track.x[1]), float(-meas.z[1])],
            [float(track.x[0]), float(meas.z[0])],
            color='blue',
            label='association'
        )
        # Set the distance label
        str_dist = "{:.2f}".format(dist)
        ax.text(
            float((-track.x[1] - meas.z[1]) / 2),
            float((track.x[0] + meas.z[0]) / 2),
            str_dist
        )
        f1 = f"Found association between track {ind_track + 1} "
        f1 += f"and measurement {ind_meas + 1} " 
        f1 += f"with Mahalanobis distance {str_dist}"
        print(f1)
        print('New association matrix:', association.association_matrix)
        print('New `unassigned_tracks`list:', association.unassigned_tracks)
        print('New `unassigned_meas` list:', association.unassigned_meas)
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
    ax.set_xlabel('y [m]')
    ax.set_ylabel('x [m]')
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
    ### Run the association with validation gating programme
    run()