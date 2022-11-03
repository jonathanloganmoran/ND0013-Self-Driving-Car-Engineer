# ------------------------------------------------------------------------------
# Exercises from Lesson 2.1: The LiDAR Sensor.
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.  
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Functions from the `Examples` section.
#
# You should have received a copy of the Udacity license with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
#
# NOTE: The current version of this programme relies on Numpy to perform data 
#       manipulation, however, a platform-specific implementation, e.g.,
#       TensorFlow `tf.Tensor` data ops, is recommended.
# ------------------------------------------------------------------------------

import cv2
import io
import math
import numpy as np
import open3d as o3d
from open3d import JVisualizer
import os
from PIL import Image
import matplotlib.pyplot as plt
from sklearn.preprocessing import RobustScaler, StandardScaler, QuantileTransformer
import sys
import zlib

### Add current working directory to path
# Alternatively, use the `pip install --editable ..` script with setuptools
sys.path.append(os.getcwd())

### Simple Waymo Open Dataset Reader library
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2
from tools.waymo_reader.simple_waymo_open_dataset_reader import label_pb2


def _object_type_name(x: int) -> str:
    """Returns the class label mapped to the input class id."""
    return label_pb2.Label.Type.Name(x)

def _laser_name(x: int) -> str:
    """Returns the LiDAR sensor name mapped to the input id."""
    return dataset_pb2.LaserName.Name.Name(x)

def _camera_name(x: int) -> str:
    """Returns the camera name mapped to the input id."""
    return dataset_pb2.CameraName.Name.Name(x)


# Example C1-5-1 : Load range image
def load_range_image(
        frame: dataset_pb2.Frame, lidar_name: int
) -> np.ndarray:
    """Returns the range image from the `frame` captured by the `lidar_name` sensor.

    :param frame: the Waymo Open Dataset `Frame` instance.
    :param lidar_name: the integer id corresponding to the
        LiDAR sensor name from which to extract and convert the range image.
    :returns: ri, the range image as a Numpy `ndarray` object.
    """

    ### Step 1 : Get the data captured by the `lidar_name` from the frame proto
    laser_data = [laser for laser in frame.lasers if laser.name == lidar_name][0]
    ### Step 2 : Get the LiDAR first return data
    # For information on multiple returns, see discussion below:
    # https://github.com/waymo-research/waymo-open-dataset/issues/45
    ri = []
    if len(laser.ri_return1.range_image_compressed) > 0:
        ri = dataset_pb2.MatrixFloat()
        ri.ParseFromString(zlib.decompress(laser.ri_return1.range_image_compressed))
        ri = np.array(ri.data).reshape(ri.shape.dims)
    return ri


# Example C1-5-6 : Convert range image to 3D point-cloud
def range_image_to_point_cloud(
        frame: dataset_pb2.Frame, lidar_name: int, vis: bool=True, inline: bool=False
) -> np.ndarray:
    """Converts a range image captured by `lidar_name` into a 3D point cloud.

    :param frame: the Waymo Open Dataset `Frame` instance.
    :param lidar_name: the integer id corresponding to the
        LiDAR sensor name from which to extract and convert the range image.
    :param vis: bool (optional), if True, the resulting point cloud will be
        displayed using the Open3D `draw_geometries` function.
    :param inline: bool (optional), If True, the output image will be
        rendered inline using Matplotlib for Jupyter notebook visualisation.
    :returns: pcl_full, the 3D point cloud instance as a Numpy `ndarray` object.
    """

    ### Extract the range image from the input frame
    ri = load_range_image(frame, lidar_name)
    # Here we clip the minimum values to 0.0 (e.g., value of -1.0 indicates no return)
    ri[ri < 0] = 0.0
    ri_range = ri[:, :, 0]
    ### Load the calibration data for this sensor
    calib_laser = [obj for obj in frame.context.laser_calibrations if obj.name == lidar_name][0]
    ### Compute the vertical beam inclinations
    height = ri_range.shape[0]
    inclination_min = calib_laser.beam_inclination_min
    inclination_max = calib_laser.beam_inclination_max
    inclinations = np.linspace(inclination_min, inclination_max, height)
    inclinations = np.flip(inclinations)
    ### Compute the azimuth angle and correct it so that the range image centre is aligned to the x-axis
    width = ri_range.shape[1]
    extrinsic = np.array(calib_laser.extrinsic.transform).reshape(4, 4)
    az_correction = math.atan2(extrinsic[1, 0], extrinsic[0, 0])
    azimuth = np.linspace(np.pi, -np.pi, width) - az_correction
    ### Expand inclination and azimuth angle s.t. every range image cell has its own appropriate value pair
    azimuth_tiled = np.broadcast_to(azimuth[np.newaxis, :], (height, width))
    inclination_tiled = np.broadcast_to(inclinations[:, np.newaxis], (height, width))
    ### Perform conversion to spherical coordinates
    x = np.cos(azimuth_tiled) * np.cos(inclination_tiled) * ri_range
    y = np.sin(azimuth_tiled) * np.cos(inclination_tiled) * ri_range
    z = np.sin(inclination_tiled) * ri_range
    ### Transform 3D points into vehicle coordinate system
    xyz_sensor = np.stack([x,y,z,np.ones_like(z)])
    xyz_vehicle = np.einsum('ij,jkl->ikl', extrinsic, xyz_sensor)
    xyz_vehicle = xyz_vehicle.transpose(1,2,0)
    ### Extract points with range value greater than 0
    idxs_range = ri_range > 0
    pcl = xyz_vehicle[idxs_range, 0:3]
    ### Visualise the point cloud
    if vis:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcl)
        if inline:
            # Return the `pcd` object to be visualised in the notebook instance
            return pcd
            # With the following usage:
            # ```
            # visualiser = JVisualizer()
            # visualiser.add_geometry(pcd)
            # visualiser.show()
            # ```
        else:
            o3d.visualization.draw_geometries([pcd])
    ### Stack the lidar point intensity values as last column
    pcl_full = np.column_stack((pcl, ri[idxs_range, 1]))    
    return pcl_full    


# Example C1-5-4 : Visualize range channel
def vis_range_channel(
        frame: dataset_pb2.Frame, lidar_name: int, inline: bool=False
):
    """Visualises the range channel captured by the `lidar_name` sensor.

    Here we crop the range image to a region of interest (ROI) about the
    x-axis at the origin with a +/- 45deg offset.

    The range values are scaled using a min-max scaling appraoch then
    converted to 8-bit values in range [0, 255].

    We assume that the LiDAR sensor has been calibrated exactly to the
    direction of motion of the ego-vehicle, i.e., that the x-axis origin
    is parallel to the direction of motion, and that the front-half of the
    range image constitutes a 180deg segment, such that a +/- 45deg shift
    from the centre x-axis makes up the 'front-facing' 90deg ROI.

    :param frame: the Waymo Open Dataset `Frame` instance.
    :param lidar_name: the integer id corresponding to the
        LiDAR sensor name to extract the range image from.
    :param inline: bool (optional), If True, the output image will be
        rendered inline using Matplotlib for Jupyter notebook visualisation. 
    """

    ### Extract the range image from the input frame
    ri = load_range_image(frame, lidar_name)
    # Here we clip the minimum values to 0.0 (e.g., value of -1.0 indicates no return)
    ri[ri < 0] = 0.0
    ### Map the value range to 8-bit grayscale
    ri_range = ri[:, :, 0]
    ri_range = ri_range * 255 / (np.amax(ri_range) - np.amin(ri_range))
    img_range = ri_range.astype(np.uint8)
    ### Cropping the range image to the ROI
    # Here we focus on +/- 45° about the x-axis origin, i.e., the image centre
    deg45 = int(img_range.shape[1] / 8)
    ri_centre = int(img_range.shape[1] / 2)
    img_range = img_range[:, ri_centre - deg45:ri_centre + deg45]
    ### Printing the maximum and minimum range values (re-scaled)
    print(f"Max. range channel value captured by '{_laser_name(lidar_name)}' sensor: {round(np.amax(img_range[:, :]), 2)}")
    print(f"Min. range channel value captured by '{_laser_name(lidar_name)}' sensor: {round(np.amin(img_range[:, :]), 2)}")
    if inline:
        plt.figure(figsize=(24, 20))
        plt.title(f"Range image captured by the '{_laser_name(lidar_name)}' sensor: the range channel")
        plt.imshow(img_range)
    else:
        cv2.imshow(f"Range image captured by the '{_laser_name(lidar_name)}' sensor: the range channel", img_range)
        cv2.waitKey(0)


# Example C1-5-3 : Retrieve maximum and minimum distance
def get_max_min_range(
        frame: dataset_pb2.Frame, lidar_name: int
):
    """Prints the minimum and maximum range given by `lidar_name`.
    
    :param frame: the Waymo Open Dataset `Frame` instance.
    :param lidar_name: the integer id corresponding to the
        LiDAR sensor name to obtain the min/max range from.
    """

    ### Extract range image from the input frame
    ri = load_range_image(frame, lidar_name)
    # Here we clip the minimum values to 0.0 (e.g., value of -1.0 indicates no return)
    ri[ri < 0] = 0.0
    ### Print the min/max range information for this sensor (in metres)
    print(f"Max. range (m) of '{_laser_name(lidar_name)}' sensor: {round(np.amax(ri[:, :, 0]), 2)}")
    print(f"Min. range (m) of '{_laser_name(lidar_name)}' sensor: {round(np.amin(ri[:, :, 0]), 2)}")


def print_range_image_shape(
        frame: dataset_pb2.Frame, lidar_name: int
):
    """Prints the shape of the range image given by `lidar_name`.

    :param frame: the Waymo Open Dataset `Frame` instance.
    :param lidar_name: the integer id corresponding to the
        LiDAR sensor name to obtain the range image shape from.
    """

    ### Load the range image from the LiDAR sensor
    ri = load_range_image(frame, lidar_name)
    ### Print the shape of the returned range image
    print(f"Range image shape captured by '{_laser_name(lidar_name)}' sensor: {ri.shape}")


# Example C1-3-3 : print angle of vertical field of view
def print_vfov_lidar(
        frame: dataset_pb2.Frame, lidar_name: int
):
    """Prints the vertical FOV of the sensor given by `lidar_name`.

    :param frame: the Waymo Open Dataset `Frame` instance.
    :param lidar_name: the integer id corresponding to the
        LiDAR sensor name to obtain the vertical FOV from.
    """

    ### Get lidar calibration data
    calib_laser = [obj for obj in frame.context.laser_calibrations if obj.name == lidar_name][0]
    ### Compute the vertical field-of-view (VFOV) in radians
    vfov_rad = calib_laser.beam_inclination_max - calib_laser.beam_inclination_min
    ### Compute and print the vertical field-of-view (VFOV) in degrees
    vfov_deg = vfov_rad * 180 / np.pi
    print(f"Vertical field-of-view (deg) of '{_laser_name(lidar_name)}' sensor: {vfov_deg}")


# Example C1-3-2 : display camera image
def display_image(
        frame: dataset_pb2.Frame, inline: bool=False
):
    """Displays the image from the camera in the frame.

    :param frame: the Waymo Open Dataset `Frame` instance.
    :param inline: bool (optional), If True, the output image will be
        rendered inline using Matplotlib for Jupyter notebook visualisation.
    """

    ### Load the camera data structure
    camera_name = dataset_pb2.CameraName.FRONT
    image = [sensor for sensor in frame.images if sensor.name == camera_name][0]
    ### Convert the actual image into rgb format
    img = np.array(Image.open(io.BytesIO(image.image)))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    ### Resize the image to better fit the screen
    dim = (int(img.shape[1] * 0.5), int(img.shape[0] * 0.5))
    resized = cv2.resize(img, dim)
    ### Display the image
    if inline:
        plt.figure(figsize=(24,20))
        plt.title(f"Image captured by the '{_camera_name(camera_name)}' camera", fontsize=20)
        plt.imshow(cv2.cvtColor(resized, cv2.COLOR_BGR2RGB))
    else:
        cv2.imshow(f"Image captured by the '{_camera_name(camera_name)}' camera", resized)
        cv2.waitKey(0)

