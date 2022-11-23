# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Define data `Association` class with a single nearest
#     neighbour association and gating based on Mahalanobis distance.
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
### Here we use the `numpy.matrix` class
# Note that this class is being deprecated and its use in this programme will be
# switched to an `numpy.ndarray` implmenetation soon.
# See: https://numpy.org/devdocs/reference/generated/numpy.matrix.html#numpy.matrix
import numpy as np
import os
from scipy.stats.distributions import chi2
import sys
from typing import List, Tuple

### Add project directory to PYTHONPATH to enable relative imports
# Alternatively, use the `pip install ..` script with setuptools
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

### Tracking package imports
# Import the tracking configuration parameters
import misc.params as params
from student.filter import Filter
from student.measurements import Measurement, Sensor
from student.trackmanagement import Track, TrackManagement


class Association(object):
    '''The Association class.

    Implements the Single Nearest Neighbor (SNN) assocation algorithm with
    validation gating.

    The SNN is a non-Bayesian approach that assumes that each track
    generates at most one measurement, and that each measurement originates
    from at most one track.

    The validation gate is a region of acceptance such that the percentage
    $100 * (1 - \alpha)$ of true measurements are rejected.

    :param association_matrix: the data association matrix used to assign
        uncertain measurements to known tracks based on Mahalanobis distance.
    :param unassigned_tracks: the list of known tracks that have not been
        assigned to a measurement.
    :param unassigned_meas: the list of uncertain measurements that have
        not been assigned to a track.
    '''

    def __init__(self):
        """Initialises the Association instance and its attributes."""

        # Instantiate the Mahalanobis distance-based association matrix
        self.association_matrix = np.matrix([])
        # Instantiate the unassigned tracks / measurements lists
        self.unassigned_tracks = []
        self.unassigned_meas = []

    def associate(self,
            track_list: List[Track],
            meas_list: List[Measurement],
            KF: Filter
    ):
        """Performs the data association step.

        :param track_list: the list of current known assigned tracks.
        :param meas_list: the list of current, already associated measurements.
        :param KF: the Kalman `Filter` instance containing the residual
            covariance function to compute matrix $\mathrm{S}$.
        """

        ### Instantiate the new unassigned tracks / measurements lists
        self.unassigned_tracks = list(range(len(track_list)))
        self.unassigned_meas = list(range(len(meas_list)))
        ### Initialise the association matrix based on Mahalanobis distance
        # Here we assume a single measurement, single track assumption
        self.association_matrix = np.matrix(
            np.full((len(track_list), len(meas_list)), fill_value=np.inf)
        )
        # Compute the association matrix values
        for x_i, track in enumerate(track_list):
            for z_j, measurement in enumerate(meas_list):
                # Compute the Mahalanobis distance
                dist = self.calc_mhd(track, measurement, KF)
                # Update the entry in the matrix with the distance value
                self.association_matrix[x_i, z_j] = dist

    def get_closest_track_and_meas(self
    ) -> Tuple[Track, Measurement]:
        """Returns the closest-distance track-measurement pair. 
        
        The shortest distance between measurement and track instances is
        obtained as the minimum entry in the association matrix. All entries in
        the association matrix correspond to every possible track-measurement
        pair. The given Mahalanobis distance score is stored in the matrix at
        the indices of the respective track / measurement positions in the
        unassigned lists. For example, the Mahalanobis distance between
        `unassigned_meas[0]` and `unassigned_track[4]` is stored in
        `association_matrix[0, 4]`.

        :returns: the track and measurement instances from the unassigned lists.
        """

        ### Find the closest track and measurement entry in matrix
        # If no valid track-measurement pairs exist in matrix, return NaN
        if np.min(self.association_matrix) == np.inf:
            return np.nan, np.nan
        else:
            # Proceed as normal, fetching distance values from matrix 
            pass
        # Get the indices of the entry with the closest distance
        idx_track, idx_measurement = np.unravel_index(
            indices=np.argmin(self.association_matrix, axis=None),
            shape=self.association_matrix.shape
        )
        ### Get the closest track and measurement objects
        track_closest = self.unassigned_tracks[idx_track]
        measurement_closest = self.unassigned_meas[idx_measurement]
        ### Remove the track and measurement objects from their lists
        self.unassigned_tracks.remove(track_closest)
        self.unassigned_meas.remove(measurement_closest)
        ### Remove the track-measurement pair from the association matrix
        self.association_matrix = np.delete(
                arr=self.association_matrix,
                obj=idx_track,
                axis=0
        )
        self.association_matrix = np.delete(
                arr=self.association_matrix,
                obj=idx_measurement,
                axis=1
        )
        ### Return the closest track-measurement pair
        return track_closest, measurement_closest   

    def gating(self,
            dist_mh: float,
            sensor: Sensor
    ) -> bool:
        """Checks if the measurement is inside the gating region of the track.

        The validation gate assumes a Gaussian measurement model with residual
        covariance $\mathrm{S}$. The area of the validation gate forms a
        hyper-ellipsoid which follows a Chi-square distribution, assuming that
        the Mahalanobis distance is a sum of squared normally-distributed
        random variables.

        The probability of a measurement being inside the gating area is given
        w.r.t. a gating threshold. This probability, $100 * (1 - p)$, can be
        obtained from the inverse Chi-square cumulative distribution function.        

        :param dist_mh: the Mahalanobis distance between track and measurement.
        :param sensor: the `Sensor` instance containing the degrees of freedom
            of the measurement space.
        :returns: boolean, whether the measurement is within the gating region
            with high certainty.
        """

        ### Compute the inverse of the Chi-square cumulative distribution func.
        #   i.e., the inverse cdf using the percent point function
        ppf = chi2.ppf(p=params.gating_threshold, df=sensor.dim_meas) 
        ### Check the given Mahalanobis distance against the limit
        if dist_mh < ppf:
            # High likelihood this is a true measurement inside the gate
            return True
        else:
            # Not likely this measurement is inside the gate
            return False

    def calc_mhd(self,
            track: Track,
            meas: Measurement,
            KF: Filter
    ) -> float:
        """Implements and returns the Mahalanobis distance calculation.

        :param track: the `Track` instance with known estimation error covariance.
        :param meas: the `Measurement` instance with uncertain position estimate
            and corresponding measurement error covariance.
        :param KF: the Kalman `Filter` instance containing the residual
            covariance function used to compute $\mathrm{S}$.
        :returns: dist, the Mahalanobis distance measure between the given
            track and the measurement.
        """

        ### Compute the measurement function for the sensor / track
        # Here we obtain the Jacobian assuming a liDAR sensor measurement
        _H = meas.sensor.get_H(track.x)
        # Computing the residual
        gamma = meas.z - meas.sensor.get_hx(track.x)
        # Computing the covariance of the residual w.r.t. measurement noise
        _S = KF.S(track, meas, _H)
        ### Calculate the Mahalanobis distance
        dist = np.matmul(gamma.T @ np.linalg.inv(_S), gamma)
        return dist
    
    def associate_and_update(self,
            manager: TrackManagement,
            meas_list: List[Measurement],
            KF: Filter
    ):
        """Performs the association and update step.

        The Kalman `Filter` instance contains the innovation / correction step
        used to update the state estimate and estimation error covariance matrix
        w.r.t. the associated measurement observed at time $t_{k}$.

        :param manager: the `TrackManagement` instance monitoring the current
            `track_list` and managing the track state / score updates.
        :param meas_list: 
        :param KF: the `Filter` instance used to perform the innovation step.
        """

        ### Associate the current tracks and given measurements
        self.associate(manager.track_list, meas_list, KF)
        ### Loop over all tracks / measurements until no more associations 
        while (
                self.association_matrix.shape[0] > 0
                and self.association_matrix.shape[1] > 0
        ):
            ### Update the associated tracks with given measurements
            # Search for the next association between a track and measurement
            idx_track, idx_measurement = self.get_closest_track_and_meas()
            if np.isnan(idx_track):
                print('---no more associations---')
                break
            else:
                # Proceed, tracks left to associate
                pass
            # Get the track at this index
            track = manager.track_list[idx_track]
            # Check sensor visibility    
            if not meas_list[0].sensor.in_fov(track.x):
                # Here we skip the update step for tracks not in the sensor FOV
                continue
            else:
                # Proceed to update tracks inside the sensor FOV
                pass
            s1 = f"Update track {track.id}"
            s1 += f" with {meas_list[idx_measurement].sensor.name}"
            s1 += f", measurement {idx_measurement}"
            print(s1)
            # Perform the Kalman update (i.e., correction / innovation) step
            KF.update(track, meas_list[idx_measurement])
            # Update the score and track state 
            manager.handle_updated_track(track)
            # Set the updated track
            manager.track_list[idx_track] = track
        ### Run the track management loop
        #   i.e., initialisation, score / state update, delete if necessary
        manager.manage_tracks(
            unassigned_tracks=self.unassigned_tracks,
            unassigned_meas=self.unassigned_meas,
            meas_list=meas_list
        )
        ### Print the resulting tracks after update
        for track in manager.track_list:            
            print('track', track.id, 'score =', track.score)