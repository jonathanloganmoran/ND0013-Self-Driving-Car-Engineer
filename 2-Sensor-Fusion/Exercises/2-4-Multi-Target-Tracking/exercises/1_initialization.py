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
### Here we use the `numpy.matrix` class
# Note that this class is being deprecated and its use in this programme will be
# switched to an `numpy.ndarray` implmenetation soon.
# See: https://numpy.org/devdocs/reference/generated/numpy.matrix.html#numpy.matrix
import numpy as np
        
        
class Measurement(object):
    '''The LiDAR sensor measurement class.

    Implements the measurement reading `z`, the measurement noise covariance `R`
    and the coordinate transform matrix `M_rot`.

    :param sens_to_veh: the coordinate transformation matrix, converts sensor
        to vehicle coordinates.
    :param z: the measurement vector, a 6D state vector where the first three
        values correspond to the estimated position in 3D. 
    :param R: the measurement noise covariance matrix.
    '''
    def __init__(self,
            gt: np.matrix, phi: np.radians, t: np.matrix
    ):
        """Initialises a new Measurement instance.

        :param gt: the ground-truth state of the vehicle defined with respect
            to the vehicle coordinate system.
        :param phi: the angle of rotation of the sensor coordinate system
            defined with respect to the vehicle coordinate system.
        :param t: the translation vector from sensor to vehicle coordinates.
        """
        
        ### Compute the rotation around the z-axis
        M_rot = np.matrix([
                    [np.cos(phi), -np.sin(phi), 0], 
                    [np.sin(phi), np.cos(phi), 0],
                    [0, 0, 1]
        ])
        ### Define the sensor-to-vehicle transformation matrix
        # Initialise a 4x4 diagonal matrix
        self.sens_to_veh = np.matrix(np.identity(n=4))            
        # Construct the sensor-to-vehicle coordinate rotation matrix
        self.sens_to_veh[0:3, 0:3] = M_rot
        # Construct the sensor-to-vehicle translation vector
        self.sens_to_veh[0:3, 3] = t
        print('Coordinate transformation matrix:', self.sens_to_veh)
        ### Transform the ground-truth state from vehicle-to-sensor coordinates
        # Define the homogeneous coordinate system
        gt_veh = np.ones((4, 1))
        # Set the first three ground-truth coordinates in world coordinate frame 
        gt_veh[0:3] = gt[0:3]
        # Perform the transformation from vehicle-to-sensor coordinates
        gt_sens = np.linalg.inv(self.sens_to_veh) * gt_veh
        ### Create a new `Measurement` object
        # Define the standard deviation for measurement noise in each axis
        sigma_lidar_x = 0.01
        sigma_lidar_y = 0.01
        sigma_lidar_z = 0.001
        # Initialise a 3x1 measurement vector
        self.z = np.zeros((3, 1))
        # Construct the x-, y- and z-axis coordinates in sensor coordinate frame
        self.z[0] = float(gt_sens[0, 0]) + np.random.normal(0, sigma_lidar_x)
        self.z[1] = float(gt_sens[1, 0]) + np.random.normal(0, sigma_lidar_y)
        self.z[2] = float(gt_sens[2, 0]) + np.random.normal(0, sigma_lidar_z)
        # Define the measurement noise covariance matrix
        self.R = np.matrix([
                    [sigma_lidar_x**2, 0, 0],
                    [0, sigma_lidar_y**2, 0], 
                    [0, 0, sigma_lidar_z**2]
        ])


class Track(object):
    '''The Track class implementing state, track id and covariance features.

    A new track is created when an unassigned measurement has been received.

    :param id: the unique track id.
    :param x: the state vector.
    :param P: the measurement error covariance matrix.
    '''

    def __init__(self,
            meas: Measurement, id: int
    ):
        """Initialises a new Track instance.

        :param meas: the received sensor measurement.
        :param id: the unique id to assign the new track.
        """
        
        print('Creating track no.', id)
        # Assign the track a unique id
        self.id = id
        # Initialise the state vector
        self.x = np.zeros((6, 1))
        # Initialise the measurement error covariance matrix
        self.P = np.zeros((6, 6))

        ############
        # TODO: initialize self.x and self.P from measurement z and R,
        #       don't forget coordinate transforms
        ############


def visualize(
        track: Track, meas: np.matrix
):
    """Displays the track and measurement info in a Matplotlib figure.


    :param track: the initialised track to visualise.
    :param meas: the measurement vector defined in sensor coordinate frame.
    """

    ### Initialise the Matplotlib figure instance
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
    ### Plot the track location and sensor measurements
    ax1.scatter(-meas.z[1], meas.z[0],
                        marker='o', color='blue', label='measurement'
    )
    ax2.scatter(-track.x[1], track.x[0],
                        color='red', s=80, marker='x', label='initialized track'
    )
    ### Transform the sensor measurement to vehicle coordinates for visualisation
    # Define the homogeneous coordinate system 
    z_sens = np.ones((4, 1))
    # Set the first three sensor coordinates in sensor coordinate frame
    z_sens[0:3] = meas.z[0:3]
    # Transform the sensor coordinates to vehicle coordinate system
    z_veh = meas.sens_to_veh * z_sens
    ### Plot the sensor measurements and track location 
    ax3.scatter(-float(z_veh[1]), float(z_veh[0]),
                        marker='o', color='blue', label='measurement'
    )
    ax3.scatter(-track.x[1], track.x[0],
                        color='red', s=80, marker='x', label='initialized track'
    )
    # Maximise the figure window
    if matplotlib.rcParams['backend'] == 'wxagg':
        mng = plt.get_current_fig_manager()
        mng.frame.Maximize(True)
    ### Define the legend and axes
    for ax in (ax1, ax2, ax3):
        ax.legend(loc='center left',
                        shadow=True, fontsize='large', bbox_to_anchor=(0.5, 0.1)
        )
        ax.set_xlabel('y [m]')
        ax.set_ylabel('x [m]')
        ax.set_xlim(-2, 2)
        ax.set_ylim(0, 2)
        ### Correct the x-axis ticks making the positive values to the left
        ticks_x = ticker.FuncFormatter(
            lambda x, pos: '{0:g}'.format(-x) if x != 0 else '{0:g}'.format(x)
        )
        ax.xaxis.set_major_formatter(ticks_x)
    ### Set the figure titles
    ax1.title.set_text('Sensor Coordinates')
    ax2.title.set_text('Vehicle Coordinates')
    txt3 = 'Vehicle Coordinates\n (track and measurement should align)'
    ax3.title.set_text(txt3)
    ### Show the figure
    plt.show()


if __name__ == '__main__':
    ### Define the ground-truth measurement vector
    gt = np.matrix([[1.7],
                    [1],
                    [0]
    ])
    ### Define the sensor translation vector and rotation angle
    t = np.matrix([[2],
                   [0.5],
                   [0]
    ])
    phi = np.radians(45)
    ### Generate a new `Measurement` instance
    meas = Measurement(gt, phi, t)
    ### Initialise a new track from this measurement
    track = Track(meas, id=1)
    ### Visualise the new track with sensor measurement
    visualize(track, meas)