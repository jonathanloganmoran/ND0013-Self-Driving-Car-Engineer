# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Define the `Camera` class and its core functionality,
#                        i.e., coordinate transforms and field of view (FOV)
#                        sanity checking for sensor fusion / track management.
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

import math
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


class Camera(object):
    '''The Camera sensor class.

    Implements the coordinate transformations between sensor and vehicle
    coordinate frames, as well as the field of view (FOV) sanity checking
    for tracking / sensor fusion tasks.

    :param fov: the opening angle (i.e., field of view) of the camera sensor.
    :param sens_to_veh: the coordinate transformation matrix from the
        sensor frame to the vehicle frame (i.e., the extrinsics matrix).
    :param veh_to_sens: the coordinate transformation matrix from the
        vehicle frame to the sensor frame.
    '''

    def __init__(self,
            phi: np.radians, t: np.ndarray
    ):
        """Initialises a Camera instance.

        :param phi: the angle between the vehicle frame and the sensor frame,
            i.e., the angle of rotation between the x-axes.
        :param t: the translation vector describing the offset along the x-axis
            of the sensor frame origin from the vehicle frame origin.
        """

        ### Defining the sensor attributes
        # The sensor field of view (i.e., the opening angle)
        self.fov = [-np.pi / 4, np.pi / 4]
        # Here we compute the rotation around the z-axis w.r.t. angle `phi`
        M_rot = np.array([
                    [np.cos(phi), -np.sin(phi), 0],
                    [np.sin(phi), np.cos(phi), 0],
                    [0, 0, 1]
        ])
        ### Defining the coordinate transformation matrices
        # Here we construct the sensor-to-vehicle transformation matrix
        self.sens_to_veh = np.array(np.identity(n=4))
        self.sens_to_veh[0:3, 0:3] = M_rot
        self.sens_to_veh[0:3, 3:] = t
        # Here we construct the vehicle-to-sensor transformation matrix
        self.veh_to_sens = np.linalg.inv(self.sens_to_veh)
    
    def in_fov(self,
            x: np.ndarray
    ) -> bool:
        """Checks if the given object `x` is within the sensor field of view.

        :param x: the object state vector to obtain the coordinates from,
            note that the position is defined w.r.t. the vehicle frame.
        :returns: boolean, whether or not the object at its position can be
            seen by the sensor, i.e., if the object is within the sensor's FOV.
        """

        ### Transform the track state position coordinates into sensor frame
        # Obtain the position coordinates of the object in vehicle frame
        _p_veh = x[0:3]
        # Convert to homogeneous coordinates
        _p_veh = np.vstack([_p_veh, np.newaxis])
        _p_veh[3] = 1
        # Construct the vehicle-to-sensor transformation
        _p_sens = self.veh_to_sens @ _p_veh
        # Obtain the position coordinates of the object in sensor frame
        p_x, p_y, _ = _p_sens[0:3]
        ### Check if the object at tracked position can be seen by the sensor
        # Calculate the angle offset of the object w.r.t. the sensor frame
        if p_x == 0:
            # Make sure that the divisor is not zero
            raise ZeroDivisionError
        alpha = math.atan(p_y / p_x)
        # Check if the angle offset is within the camera opening angle
        if np.abs(alpha) <= self.fov[1]:
            return True
        else:
            return False
        

def run():
    """Tests the camera visibility function and plots the results.

    A `Camera` instance is created with a set of attribute values specifying its
    translation vector and angle of rotation. Inside the `Camera` class is the
    definition of the sensor field of view used to check whether or not an object
    lies within the camera field of view. Here the object state is initialised
    with simulated track position and velocity values.
    """

    ### Defining the camera parameters
    # The translation vector
    t = np.array([[2],
                  [0],
                  [0]
    ])
    # The angle of rotation between vehicle and sensor frame
    phi = np.radians(45)
    ### Creating a new `Camera` instance
    cam = Camera(phi, t)
    ### Initialise the Matplotlib figure
    fig, ax = plt.subplots(1, 1,figsize=(24, 20),
                    constrained_layout=True
    )
    # Setting the figure title
    txt1 = 'Track Positions With Respect to Camera Field of View'
    fig.suptitle(txt1, fontsize=20)
    ### Running the programme loop over the simulated track detections
    for i in range(50):
        # Define the track state vector, i.e., position and velocity estimate
        # Note that the position is defined w.r.t. the vehicle frame
        x = np.array([
                [np.random.uniform(-5, 5)],     # Position along x-axis
                [np.random.uniform(-5, 5)],     # Position along y-axis
                [0],                            # Position along z-axis
                [0],                            # Velocity along x-axis
                [0],                            # Velocity along y-axis
                [0]                             # Velocity along z-axis
        ])
        ### Check if the position of tracked object `x` is visible by the camera
        result = cam.in_fov(x)
        ### Plot the results
        # Define the homogeneous coordinate system
        pos_veh = np.ones((4, 1))
        # Obtain the position estimate from the track state vector
        pos_veh[0:3] = x[0:3] 
        # Construct the vehicle-to-sensor coordinate transformation 
        pos_sens = cam.veh_to_sens @ pos_veh
        if result == True:
            ### If the position is within the camera field of view
            # Plot the track position with a marker of colour `col`
            col = 'green'
            ax.scatter(float(-pos_sens[1]), float(pos_sens[0]),
                            marker='o', s=95, color=col, label='Visible track'
            )
        else:
            ### If the position is not within the camera field of view
            # Plot the track position with a marker of colour `col`
            col = 'red'
            ax.scatter(float(-pos_sens[1]), float(pos_sens[0]),
                            marker='o', s=95, color=col, label='Invisible track'
            )
            # Plot the track `in_fov` result
        ax.text(
                x=float(-pos_sens[1]),
                y=float(pos_sens[0]),
                s=str(result),
                clip_on=True    # Here we clip any values outside the plot limit
        ) 
    ### Plot the field of view of the camera sensor
    ax.plot([0, -5], [0, 5], color='blue', label='Field of view') 
    ax.plot([0, 5], [0, 5], color='blue')
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
    # Initialise the legend instance
    ax.legend(handle_list, label_list, loc='center left',
                        shadow=True, fontsize='large', bbox_to_anchor=(0.9, 0.1)
    )
    # Set the axes labels
    ax.set_xlabel('y [m]', fontsize=14)
    ax.set_ylabel('x [m]', fontsize=14)
    # Set the x-axis and y-axis limits
    ax.set_xlim(-5, 5)
    ax.set_ylim(0, 5)
    # Correct the x-axis ticks so that the positive values are to the left
    ticks_x = ticker.FuncFormatter(
        lambda x, pos: '{0:g}'.format(-x) if x != 0 else '{0:g}'.format(x)
    )
    ax.xaxis.set_major_formatter(ticks_x)
    ### Show the resulting plot
    if matplotlib.rcParams['backend'] != 'agg':
        plt.show()


if __name__ == '__main__':
    ### Run the programme loop
    run()