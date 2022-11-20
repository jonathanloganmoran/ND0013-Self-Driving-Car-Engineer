# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Define the `Track` and `TrackManagement` classes
#                        and their core functionality, i.e., the track state,
#                        track id, covariance matrices, and functions for
#                        track initialisation / deletion.
#                        
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
import collections
import numpy as np

### Add project directory to PYTHONPATH to enable relative imports
# Alternatively, use the `pip install ..` script with setuptools
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(
    os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__)))
)
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

### Import the track management parameters
import misc.params as params 


class Track(object):
    '''The Track class.

    :param id: the unique track id.
    :param state: the string of the current state determined by the track score.
    :param score: the track score as computed by a n-last detections weighting.
    :param x: the state vector containing the position estimate.
    :param P: the estimation error covariance matrix.
    :param width: the width from the measurement associated with this track.
    :param height: the height from the measurement associated with this track.
    :param length: the length from the measurement associated with this track.
    :param yaw: the rotation from sensor-to-vehicle coordinates.
    :param t: the timestamp of the measurement w.r.t. elapsed time `dt`.
    '''

    def __init__(self,
            meas: Measurement,
            _id: int
    ):
        """Initialises a new Track instance.

        :param meas: the LiDAR measurement to associate with this track.
        :param id: the unique id to assign this track.
        """

        print('creating track no.', _id)
        # Obtain the rotation matrix from sensor-to-vehicle coordinates
        M_rot = meas.sensor.sens_to_veh[0:3, 0:3]        
        ### Initialise the track state vector
        # Initialise the 6x1 state vector
        self.x = np.ones((6, 1))
        # Fetch the unassigned measurement vector
        _z_sens = meas.z
        # Convert to homogeneous coordinate system
        _z_sens = np.vstack((_z_sens, np.new_axis))
        _z_sens[3] = 1
        # Obtain the sensor-to-vehicle transformation matrix
        _T_sens2veh = meas.sensor.sens_to_veh
        # Convert coordinates from sensor-to-vehicle frame
        self.x[0:4] = _T_sens2veh @ _z_sens
        ### Initialise the estimation error covariance matrix
        self.P = np.zeros((6, 6))
        # Obtain the sensor-to-vehicle rotation matrix
        _M_rot = meas.sensor.sens_to_veh[0:3, 0:3]
        # Fetch the measurement noise covariance
        _R_sens = meas.R
        # Construct the position estimation error covariance
        self.P[0:3, 0:3] = np.matmul(_M_rot @ _R_sens, _M_rot.T)
        # Initialise the velocity estimation error covariance
        self.P[3:6, 3:6] = np.array(np.identity(n=3))  
        # Set the velocity estimation covariance values along the diagonal
        # to something large, since we cannot directly measure velocity;
        # Here we set the estimation error covariance entries for velocity in 3D
        self.P[3, 3] = sigma_p44**2
        self.P[4, 4] = sigma_p55**2
        self.P[5, 5] = sigma_p66**2
        # Set the track state to the starting state
        self.state = 'initialised'
        # Set the track score
        self.score = 1. / params.window
        ### Initialising the other track attributes
        # Set the track id
        self.id = _id
        # Set the measurement width
        self.width = meas.width
        # Set the measurement length
        self.length = meas.length
        # Set the measurement height
        self.height = meas.height
        # Compute the rotation angle from sensor-to-vehicle coordinates
        # i.e., the roll around the x-axis w.r.t. the vehicle frame
        self.yaw =  np.arccos(
            M_rot[0, 0] * np.cos(meas.yaw) + M_rot[0,1] * np.sin(meas.yaw)
        )
        # Set the timestamp of the measurement
        # i.e., the (frame number - 1) * dt
        self.t = meas.t

    def set_x(self,
            x: np.matrix
    ):
        """Sets the state vector estimate to the given object."""
        self.x = x
        
    def set_P(self,
            P: np.matrix
    ):
        """Sets the estimation error covariance to the given object."""
        self.P = P  
        
    def set_t(self,
            t: np.matrix
    ):
        """Sets the translation vector instance to the given object."""
        self.t = t

    def update_attributes(self,
            meas: Measurement
    ):
        """Updates the track with the latest measurement.

        Uses the exponential sliding average to compute the estimate of the
        dimensions and orientation of the object from the provided measurement.
        
        :param meas: the Measurement instance to update the track state with.
        """

        ### Estimate the dimensions and the orientation for LiDAR measurement
        # Using exponential sliding average
        if meas.sensor.name == 'lidar':
            c = params.weight_dim
            self.width = c * meas.width + (1 - c) * self.width
            self.length = c * meas.length + (1 - c) * self.length
            self.height = c * meas.height + (1 - c) * self.height
            # Tranform the measurement coordinates from sensor-to-vehicle
            M_rot = meas.sensor.sens_to_veh
            self.yaw = np.arccos(
                M_rot[0, 0] * np.cos(meas.yaw) + M_rot[0, 1] * np.sin(meas.yaw)

            )


class TrackManagement(object):
    '''The Track Management class.

    :param N: the current number of tracks managed by this instance.
    :param track_list: the list of current (active) tracks.
    :param result_list: the 
    :param last_id: the track id of the last track to be added.

    '''

    def __init__(self):
        """Initialises a new TrackManagement instance."""

        ### Initialise the attributes
        # Set the number of current managed tracks to zero 
        self.N = 0
        # Instantiate the empty track / result lists
        self.track_list = []
        self.result_list = []
        # Set the last track id inserted to void
        self.last_id = -1
        
    def manage_tracks(self,
            unassigned_tracks: List[Track],
            unassigned_meas: List[Measurement],
            meas_list: List[Measurement]
    ):
        """Runs the track management loop.

        In the initialisation loop, a new track is created for each new
        LiDAR measurement. Here we choose to skip the initialisation of new
        tracks for camera measurements, as their position in 3D cannot be
        accurately estimated from a single time-step.

        In the management loop, all unassigned tracks are sweeped and their
        track scores are updated based on the position estimate relative to
        the visibility of the current sensor. If any measurements are outside
        of the sensor visibility, the track score will remain the same.
        In other words, we choose to avoid decreasing the score of tracks
        who are not currently visibile by both of the sensor types. 

        :param unassigned_tracks: the list of tracks that have not yet been
            assigned to a measurement.
        :param unassigned_meas: the list of measurements that have not yet been
            associated with a track.
        :param meas_list: the list of measurements from this current time-step.
        """
        ############
        # TODO Step 2: implement track management:
        # - decrease the track score for unassigned tracks
        # - delete tracks if the score is too low or P is too big
        #   (check params.py for parameters that might be helpful, but
        # feel free to define your own parameters)
        ############

        ### Instantiate a list of tracks to delete
        tracks_to_delete = []
        ### Loop through all unassigned tracks
        for i in unassigned_tracks:
            # Get the next track in the list
            track = self.track_list[i]
            ### Decrease the track score for any unassigned tracks
            if meas_list:
                # Check sensor visibility based on track's last measurement
                if meas_list[0].sensor.in_fov(track.x):
                    # Decrease track score if measurement was last within FOV
                    track.score -= 1. / params.window
                    # Prevent the track score from dropping below `0`
                    track.score = max(track.score, 0.)
                    ### Delete track according to threshold
                    # Obtain the delete threshold to use based on track state
                    if track.state == 'confirmed':
                        threshold = params.delete_threshold
                    elif track.state in ['initialized', 'tentative']:
                        # TODO: move into `params` file
                        threshold = 0.17
                    else:
                        # Track state not recognised
                        raise ValueError(f"Invalid track state '{track.state}'")
                    # Delete track if track score less than threshold
                    # or if the estimation error covariance is too high
                    if (
                        track.score < threshold
                        or track.P[0, 0] > params.max_P
                        or track.P[1, 1] > params.max_P
                    ):
                        tracks_to_delete.append(track)
        ### Delete all tracks marked for deletion during sweep
        _ = [self.delete_track(track) for track in tracks_to_delete]
        ### Initialise a new track for each unassigned measurement
        for j in unassigned_meas:
            # Here we only initialise new tracks for LiDAR measurements
            # Since we cannot accurately estimate 3D position from camera
            # measurements in a single time-step
            if meas_list[j].sensor.name == 'lidar':
                self.init_track(meas_list[j])


    def add_track_to_list(self,
            track: Track
    ):
        """Adds the given track to the track manager.

        When a new track is added to the track list, the number of
        tracks `N` increments by one. The `last_id` attribute is also
        updated to reflect the `id` of the given track added to the list.

        :param track: the new track to add to the track list.
        """

        ### Add the new track to the current tracks list
        self.track_list.append(track)
        # Increment the current number of tracks managed
        self.N += 1
        # Update the last inserted track id
        self.last_id = track.id

    def delete_track(self,
            track: Track
    ):
        """Removes the given track from the track list.

        :param track: the track instance to remove from the track list.
        """

        print('deleting track no.', track.id)
        self.track_list.remove(track)
        
    def handle_updated_track(self,
            track: Track
    ):
        """Updates the given track's score and state.

        :param track: the specific track to update.
        """
        ############
        # TODO Step 2: implement track management for updated tracks:
        # - increase track score
        # - set track state to 'tentative' or 'confirmed'
        ############
        pass
        ############
        # END student code
        ############

    def init_track(self,
            meas: Measurement
    ):
        """Initialises a new track instance and adds it to the track list.

        :param meas: the LiDAR measurement to assign to a new track instance.
        """

        ### Create a new track instance
        # Incrementing the last inserted track id by one
        track = Track(meas, self.last_id + 1)
        # Add the new track to the list
        self.add_track_to_list(track)