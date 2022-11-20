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
from typing import List

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


class Association:
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
            covariance matrix `S`.
        """

        ############
        # TODO Step 3: association:
        # - replace association_matrix with the actual association matrix based
        #   on Mahalanobis distance (see below) for all tracks and measurements
        # - update list of unassigned measurements and unassigned tracks
        ############
        
        ### Initialise the association matrix based on Mahalanobis distance
        # Here we assume a single measurement, single track assumption
        # Instantiate the new association matrix
        self.association_matrix = np.matrix([])
        # Instantiate the new unassigned tracks / measurements lists
        self.unassigned_tracks = []
        self.unassigned_meas = []
        # Reset the current measurement and track lists
        if len(meas_list) > 0:
            self.unassigned_meas = [0]
        if len(track_list) > 0:
            self.unassigned_tracks = [0]
        if len(meas_list) > 0 and len(track_list) > 0: 
            self.association_matrix = np.matrix([[0]])
        ############
        # END student code
        ############

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
        ############
        # TODO Step 3: find closest track and measurement:
        # - find minimum entry in association matrix
        # - delete row and column
        # - remove corresponding track and measurement from unassigned_tracks
        #   and unassigned_meas
        # - return this track and measurement
        ############
        # the following only works for at most one track and one measurement
        update_track = 0
        update_meas = 0
        
        # remove from list
        self.unassigned_tracks.remove(update_track) 
        self.unassigned_meas.remove(update_meas)
        self.association_matrix = np.matrix([])
            
        ############
        # END student code
        ############ 
        return update_track, update_meas     

    def gating(self,
            dist_mh: float,
            sensor: Sensor
    ) -> bool:
        """Checks if the measurement is inside the gating region of the track.

        :param dist_mh: the Mahalanobis distance between track and measurement.
        :param sensor: the `Sensor` instance containing the sensor-to-vehicle
            transformation matrix used to convert the residual covariance to
            vehicle space.
        :returns: boolean, whether the measurement is within the gating region.
        """

        ############
        # TODO Step 3: return True if measurement lies inside gate, otherwise False
        ############
        pass
        ############
        # END student code
        ############ 
        
    def calc_mhd(self,
            track: Track,
            meas: Measurement,
            KF: Filter
    ) -> float:
        """Implements and returns the Mahalanobis distance calculation.

        :param track: the `Track` instance with known estimation error covariance.
        :param meas: the `Measurement` instance with uncertain position estimate
            and corresponding measurement error covariance.
        :returns: dist_mh, the Mahalanobis distance measure between the given
            track and the measurement.
        """

        ############
        # TODO Step 3: calculate and return Mahalanobis distance
        ############
        pass
        ############
        # END student code
        ############ 
    
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
            # Get the track at this index
            track = manager.track_list[idx_track]
            # Check sensor visibility    
            if not meas_list[0].sensor.in_fov(track.x):
                # Here we skip the update step for tracks not in the sensor FOV
                continue
            # Perform the Kalman update (i.e., correction / innovation) step
            s1 = f"Update track {track.id} with {meas_list[idx_meas].sensor.name}"
            s1 += f", measurement {idx_measurement}"
            print(s1)
            KF.update(track, meas_list[idx_meas])
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