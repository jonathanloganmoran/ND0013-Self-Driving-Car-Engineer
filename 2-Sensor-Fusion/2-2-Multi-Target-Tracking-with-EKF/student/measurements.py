# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Define the `Sensor` and `Measurement` classes
#                        and their core functionality, i.e., the measurement
#                        vector, covariance matrices, coordinate transformations
#                        and camera intrinsic parameters.
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
### Here we use the `numpy.matrix` class
# Note that this class is being deprecated and its use in this programme will be
# switched to an `numpy.ndarray` implmenetation soon.
# See: https://numpy.org/devdocs/reference/generated/numpy.matrix.html#numpy.matrix
import numpy as np
import os
import sys
from typing import Union

### Simple Waymo Open Dataset Reader library
# Used for the `CameraCalibration` and `LidarCalibration` typing hints
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2

### Add project directory to PYTHONPATH to enable relative imports
# Alternatively, use the `pip install ..` script with setuptools
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(
    os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__)))
)
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

### Measurement model imports
# Importing the camera / LiDAR sensor configuration parameters
import misc.params as params


class Sensor(object):
    '''The Sensor class.

    :param name: the name of this sensor instance, can be one of
        ['camera', 'lidar']. 
    :param dim_meas: the dimensions of the measurement space.
    :param sens_to_veh: the sensor-to-vehicle transformation matrix
        used to convert sensor coordinates to the vehicle frame.
    :param veh_to_sens: the vehicle-to-sensor transformation matrix
        used to convert vehicle coordinates to the sensor frame.
    :param fov: the angular field of view of this sensor instance,
        defined as an interval with values given in radians. 
    :param f_i: 'camera' only, the i-coordinate of the focal length.
    :param f_j: 'camera' only, the j-coordinate of the focal length.
    :param c_j: 'camera' only, the j-coordinate of the principal point.
    '''

    def __init__(self,
            name: str,
            calib: Union[
                dataset_pb2.CameraCalibration, dataset_pb2.LidarCalibration
            ]
    ):
        """Initialises a new Sensor instance.
        
        :param name: the string containing the sensor type, can be one of
            ['camera', 'lidar'].
        :param calib: the protobuf container corresponding to the sensor
            calibration attributes, i.e., the intrinsic and extrinsic
            matrices in addition to either: `width`, `height` for the 'camera'
            sensor, or `beam_inclinations`, `beam_inclination_min`,
            `beam_inclination_max` for the 'lidar' sensor.
        """

        ### Setting the sensor attributes
        # Set the sensor type to the given name
        self.name = name
        if name == 'lidar':
            ### Configuring the LiDAR sensor type
            # Set the number of dimensions of the measurement space
            self.dim_meas = 3
            # Initialise the sensor-to-vehicle transformation matrix
            # Here the identity matrix is used since LiDAR measurements are
            # already given in vehicle coordinate frame
            self.sens_to_veh = np.matrix(
                    np.identity(n=4)
            )
            # The angular field of view of the LidAR sensor (radians)
            self.fov = [-np.pi / 2, np.pi / 2]
        elif name == 'camera':
            ### Configuring the RGB camera sensor type
            # The number of dimensions of the measurement space
            self.dim_meas = 2
            # The sensor-to-vehicle transformation matrix
            # Here we convert the extrinsic calibration matrix to Numpy `matrix`
            self.sens_to_veh = np.matrix(
                    calib.extrinsic.transform
            ).reshape(4, 4)
            # The focal length
            self.f_i = calib.intrinsic[0]
            self.f_j = calib.intrinsic[1]
            # The principal point
            self.c_i = calib.intrinsic[2]
            self.c_j = calib.intrinsic[3]
            # The angular field of view of the camera sensor (radians)
            # Here we limit the boundary region for greater accuracy 
            self.fov = [-0.35, 0.35]
        else:
            raise ValueError(f"Invalid sensor type '{name}'")
        ### Obtain the vehicle-to-sensor transformation matrix    
        self.veh_to_sens = np.linalg.inv(self.sens_to_veh)


    def in_fov(self,
            x: Union[np.ndarray, np.matrix]:
    ): -> bool
        """Checks if the given object `x` is within the sensor field of view.

        :param x: the object state vector to obtain the coordinates from,
            note that the position is defined w.r.t. the vehicle frame.
        :returns: boolean, whether or not the object at its position can be
            seen by the sensor, i.e., if the object is within the sensor's FOV.
        """

        ### Transform the track state position into sensor frame
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
            e1 = f"Invalid coordinates (sensor frame) '{_p_sens.tolist()}'"
            raise ZeroDivisionError(e1) 
        alpha = math.atan(p_y / p_x)
        # Check if the angle offset is within the camera opening angle
        # Not assuming the FOV is symmetric
        if np.min(self.fov) <= alpha <= np.max(self.fov):
            # Object is within view of the sensor
            return True
        else:
            # Object is not in view of the sensor
            return False
             
    def get_hx(self,
            x: Union[np.ndarray, np.matrix]
    ) -> np.matrix:
        """Implements the non-linear camera measurement function.

        :param x: the track state vector used to calculate the expectation value.
            value.
        :returns: hx, the non-linear camera measurement function evaluated at
            the given `x`.
        """

        ### Calculate the non-linear measurement expectation value
        # Here the LiDAR sensor model is linear, so we transform the measurement
        # into homogeneous coordinates in sensor frame
        if self.name == 'lidar':
            # Construct the homogeneous coordinate system
            _p_veh = np.vstack([x[0:3], np.newaxis])
            _p_veh[3] = 1 
            # Transform from vehicle-to-sensor coordinates
            _p_sens = self.veh_to_sens @ _p_veh
            # Return the position estimate in LiDAR coordinate frame
            return _p_sens[0:3]
        elif self.name == 'camera':
            ############
            # TODO Step 4: implement nonlinear camera measurement function h:
            # - transform position estimate from vehicle to camera coordinates
            # - project from camera to image coordinates
            # - make sure to not divide by zero, raise an error if needed
            # - return h(x)
            ############
            # Construct the homogeneous coordinate system
            _p_veh = np.vstack([x[0:3], np.newaxis])
            _p_veh[3] = 1
            # Transform from vehicle-to-sensor coordinates
            _p_sens = self.veh_to_sens @ _p_veh
            # Project into the image space
            if _p_sens[0, 0] == 0:
                # Make sure that the divisor is not zero
                e1 = f"Invalid coordinates (sensor frame) '{_p_sens.tolist()}'"
                raise ZeroDivisionError(e1)
            else:
                # Project the coordinates into the image space
                i = self.c_i - self.f_i * _p_sens[1, 0] / _p_sens[0, 0]
                j = self.c_j - self.f_j * _p_sens[2, 0] / _p_sens[0, 0]
                return np.matrix([[i], [j]])
    
    def get_H(self,
            x: Union[np.ndarray, np.matrix]
    ) -> Union[np.ndarray, np.matrix]:
        """Implements the linearised camera measurement function.

        The non-linear camera measurement function $h(\mathrm{x})$ is linearised
        through function approximation with the first-order Taylor series
        evaluated at the extrapolation point `x`. We assume that the linear
        approximation tangent to $h(\mathrm{x})$ has a slope around
        $\mu \approx \mathrm{x}$ such that the higher-order terms become zero.

        The Jacobian $\mathrm{H}_{J}$ implemented here contains the partial
        derivatives of the multi-dimensional function $h(\mathrm{x})$ needed to
        compute the multi-variate Taylor expansion.

        :param x: the position estimate obtained from the track state vector.
        :returns: the Jacobian of the non-linear camera measurement model.
        """

        ### Calculate the Jacobian at the expansion point `x`
        # Initialise the Jacobian
        H = np.matrix(np.zeros((self.dim_meas, params.dim_state)))
        # Obtain the rotation matrix
        R = self.veh_to_sens[0:3, 0:3]
        # Obtain the translation vector
        T = self.veh_to_sens[0:3, 3]
        if self.name == 'lidar':
            H[0:3, 0:3] = R
        elif self.name == 'camera':
            # Check if divide by zero exists
            if (
                R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0] == 0
            ):
                # Raise the error and return the vector `x` for debugging
                e1 = f'Jacobian not defined for this {x.tolist()}!'
                raise ZeroDivisionError(e1)
            else:
                H[0, 0] = self.f_i * (-R[1, 0] / (
                    R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    ) + R[0, 0] * (
                        R[1, 0] * x[0] + R[1, 1] * x[1] + R[1, 2] * x[2] + T[1]
                        ) + R[0, 0] * (
                        R[1, 0] * x[0] + R[1, 1] * x[1] + R[1,2] * x[2] + T[1]
                        ) / ((
                        R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    )**2)
                )
                H[1, 0] = self.f_j * (-R[2, 0] / (
                    R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    ) + R[0, 0] * (
                        R[2, 0] * x[0] + R[2, 1] * x[1] + R[2, 2] * x[2] + T[2]
                        ) / ((
                        R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    )**2)
                )
                H[0, 1] = self.f_i * (-R[1, 1] / (
                    R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    ) + R[0, 1] * (
                        R[1, 0] * x[0] + R[1, 1] * x[1] + R[1, 2] * x[2] + T[1]
                    ) / ((
                        R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    )**2)
                )
                H[1, 1] = self.f_j * (-R[2, 1] / (
                    R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    ) + R[0, 1] * (
                        R[2, 0] * x[0] + R[2, 1] * x[1] + R[2, 2] * x[2] + T[2]
                    ) / ((
                        R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    )**2)
                )
                H[0, 2] = self.f_i * (-R[1, 2] / (
                    R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    ) + R[0, 2] * (
                    R[1, 0] * x[0] + R[1, 1] * x[1] + R[1, 2] * x[2] + T[1]
                    ) / ((
                        R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    )**2)
                )
                H[1, 2] = self.f_j * (-R[2, 2] / (
                    R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    ) + R[0, 2] * (
                    R[2, 0] * x[0] + R[2, 1] * x[1] + R[2, 2] * x[2] + T[2]
                    ) / ((
                    R[0, 0] * x[0] + R[0, 1] * x[1] + R[0, 2] * x[2] + T[0]
                    )**2)
                )
        return H   

    def generate_measurement(self,
            num_frame: int,
            z: Union[np.ndarray, np.matrix],
            meas_list: List[Measurement]
    ) -> List[Measurement]:
        """Initialises a new Measurement instance and returns it in a list.

        :param num_frame: the frame id from which this measurement was captured.
        :param z: the raw measurement vector.
        :param meas_list: the list of measurements to update with this object.
        :returns: meas_list, the updated measurement list containing the new
            measurement object created from sensor reading `z`.  
        """

        ### Generate a new measurement
        if self.name in {'lidar', 'camera'}:
            # Create a new Measurement instance with reference to Sensor self
            meas = Measurement(num_frame, z, self)
            # Add the measurement to the list
            meas_list.append(meas)
            # Return the updated list
            return meas_list
        else:
            raise ValueError(f"Invalid sensor type '{self.name}'")

        
class Measurement(object):
    '''The Measurement class.

    :param t: the measurement timestamp w.r.t. the current frame id and
        the elapsed time `dt`.
    :param sensor: the Sensor instance from which this measurement was
        generated, its `name` attribute can be one of ['camera', 'lidar'].
    :param z: the measurement vector containing the position and velocity
        estimates as measured by the sensor.
    :param R: the measurement noise covariance matrix.
    :param width: the estimated width of the measured object.
    :param length: the estimated length of the measured object.
    :param height: the estimated height of the measured object.
    :param yaw: the heading angle of the bounding box in radians, i.e.,
        the angle required to rotate +x to the surface normal of the box
        normalised to [-pi, pi).
    '''

    def __init__(self,
            num_frame: int,
            z: Union[np.ndarray, np.matrix],
            sensor: Sensor
    ):
        """Initialises a new Measurement instance.

        :param num_frame: the frame id from which this measurement was captured.
        :param z: the raw measurement vector.
        :param sensor: the Sensor instance from which this measurement was
            generated, its `name` attribute can be one of ['camera', 'lidar'].
        """

        ### Create the measurement instance
        # Set the timestamp
        self.t = (num_frame - 1) * params.dt
        # Set the sensor type that generated this measurement
        self.sensor = sensor
        ### Configure the sensor-specific parameters
        if sensor.name == 'lidar':
            # Load the LiDAR standard deviation values
            sigma_lidar_x = params.sigma_lidar_x
            sigma_lidar_y = params.sigma_lidar_y
            sigma_lidar_z = params.sigma_lidar_z
            # Initialise the measurement vector
            self.z = np.zeros((sensor.dim_meas, 1))
            # Set the state vector position estimate
            self.z[0] = z[0]
            self.z[1] = z[1]
            self.z[2] = z[2]
            # Initialise the measurement noise covariance matrix 
            self.R = np.matrix([
                [sigma_lidar_x**2, 0., 0.],
                [0., sigma_lidar_y**2, 0.], 
                [0., 0., sigma_lidar_z**2]
            ])
            # Set the estimates
            self.width = z[4]
            self.length = z[5]
            self.height = z[3]
            # Set the yaw angle
            self.yaw = z[6]
        elif sensor.name == 'camera':
            # Load the camera standard deviation values
            sigma_cam_i = params.sigma_cam_i
            sigma_cam_j = params.sigma_cam_j
            # Initialise the measurement vector
            self.z = np.zeros((sensor.dim_meas, 1))
            # Set the state vector position estimate
            self.z[0] = z[0]
            self.z[1] = z[1]
            # Initialise the measurement noise covariance matrix
            self.R = np.matrix([
                [sigma_cam_i**2, 0.],
                [0., sigma_cam_j**2]
            ])
            # Set the estimates of the bounding box dimensions
            self.width = z[2]
            self.length = z[3]
        else:
            raise ValueError(f"Invalid sensor type '{sensor.name}'")