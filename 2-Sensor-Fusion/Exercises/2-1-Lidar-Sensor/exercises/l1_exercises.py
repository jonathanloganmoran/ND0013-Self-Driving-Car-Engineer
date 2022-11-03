# ------------------------------------------------------------------------------
# Exercises from Lesson 2.1: The LiDAR Sensor.
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.  
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Starter code for the `Exercises` section.
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
import matplotlib.pyplot as plt
import numpy as np
import os
from PIL import Image
import sys
import zlib

### Add current working directory to path
# Alternatively, use the `pip install --editable ..` script with setuptools
sys.path.append(os.getcwd())

### Simple Waymo Open Dataset Reader library
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2
from tools.waymo_reader.simple_waymo_open_dataset_reader import label_pb2

from l1_examples import load_range_image


def _object_type_name(x: int) -> str:
    """Returns the class label mapped to the input class id."""
    return label_pb2.Label.Type.Name(x)

def _laser_name(x: int) -> str:
    """Returns the LiDAR sensor name mapped to the input id."""
    return dataset_pb2.LaserName.Name.Name(x)

def _camera_name(x: int) -> str:
    """Returns the camera name mapped to the input id."""
    return dataset_pb2.CameraName.Name.Name(x)


# Exercise C1-5-5 : Visualize intensity channel
def vis_intensity_channel(
    frame: dataset_pb2.Frame, lidar_name: int, inline: bool=False
):
    """Visualises the intensity channel of the range image.

    Here we crop the range image captured by the `lidar_name` sensor to a
    region of interest (ROI) about the x-axis origin a +/- 45deg offset.

    The intensity values are scaled using a heuristics-based approach,
    i.e., a contrast adjustment is performed, which multiplies all
    intensity values by one-half the maximum intensity value. This was
    selected over traditional scaling methods, e.g., z-normalization,
    as the contrast adjustment used here mitgates the influence of
    intensity outliers while avoiding significant increases in noise.

    We assume that the LiDAR sensor has been calibrated exactly to the
    direction of motion of the ego-vehicle, i.e., that the x-axis origin
    is parallel to the direction of motion, and that the front-half of the
    range image constitutes a 180deg segment, such that a +/- 45deg shift
    from the centre x-axis makes up the 'front-facing' 90deg ROI.

    :param frame: the Waymo Open Dataset `Frame` instance.
    :param lidar_name: the integer id corresponding to the
        LiDAR sensor name from which to extract the range image.
    :param inline: bool (optional), If True, the output image will be
        rendered inline using Matplotlib for Jupyter notebook visualisation. 
    """

    print("Exercise C1-5-5")
    ### Step 1 : Extract the range image from the frame
    ri = load_range_image(frame, lidar_name)
    # Setting the invalid (no return) values to 0.0
    ri[ri < 0] = 0.0
    ### Step 2 : Map the intensity value range to 8-bit grayscale
    # Extract the intensity values from the range image
    ri_intensity = ri[:, :, 1]
    # Perform a contrast adjustment over the intensity values
    ri_scaled = ri_intensity * np.amax(ri_intensity) / 2
    # Fit to 8-bit grayscale
    ri_intensity = ri_scaled * 255 / (np.amax(ri_intensity) - np.amin(ri_intensity))
    ri_intensity = ri_intensity.astype(np.uint8)
    ### Step 3 : Cropping the range image to the desired ROI
    # Here we focus on +/- 45° about the x-axis origin, i.e., the image centre
    deg45 = int(ri_intensity.shape[1] / 8)
    ri_centre = int(ri_intensity.shape[1] / 2)
    ri_intensity = ri_intensity[:, ri_centre - deg45:ri_centre + deg45]
    ### Step 4 : Printing the maximum and minimum intensity values
    print(f"Max. intensity channel value captured by '{_laser_name(lidar_name)}' sensor: {round(np.amax(ri_intensity[:, :]), 2)}")
    print(f"Min. intensity channel value captured by '{_laser_name(lidar_name)}' sensor: {round(np.amin(ri_intensity[:, :]), 2)}")
    ### Step 5 : Displaying the resulting intensity channel of the range image
    if inline:
        plt.figure(figsize=(24, 20))
        plt.title(f"Range image captured by the '{_laser_name(lidar_name)}' sensor: the intensity channel")
        plt.imshow(img_range)
    else:
        cv2.imshow(f"Range image captured by the '{_laser_name(lidar_name)}' sensor: the intensity channel", ri_intensity)
        cv2.waitKey(0)


# Exercise C1-5-2 : Compute pitch angle resolution
def print_pitch_resolution(
        frame: dataset_pb2.Frame, lidar_name: int
):
    """Prints the vertical angular resoution of the range image.

    Using the equally-divided elevation angle formula from the 
    Projection By Elevation Angle (PBEA) method, such that:
    $$
    \begin{align}
        \theta_{res} = (\theta_{up} - \theta_{down}) / h, \\
    \end{align}
    $$
    for maximum and minimum elevation angles $\theta_{up}$ and $\theta{down}$,
    respectively, and for height $h$ of the range image.

    :param frame: the Waymo Open Dataset `Frame` instance.
    :param lidar_name: the integer id corresponding to the
        LiDAR sensor name from which to print the pitch resolution.
    """

    print("Exercise C1-5-2")
    ### Load the range image
    ri = load_range_image(frame, lidar_name)
    ### Compute vertical field-of-view (VFOV) from lidar calibration 
    # Get the laser calibration data
    calib_laser = [sensor for sensor in frame.context.laser_calibrations if sensor.name == lidar_name][0] 
    # Get the maximum and minimum pitch elevation angles
    pitch_max = calib_laser.beam_inclination_max
    pitch_min = calib_laser.beam_inclination_min
    # Compute the vertical resolution in angular degrees
    vfov = pitch_max - pitch_min
    pitch_res_rad = vfov / ri.shape[0]
    pitch_res_deg = pitch_res_rad * 180 / np.pi
    # print(f'Pitch angle resolution: {pitch_res_deg:.2f}°')
    # Convert to angular minutes
    pitch_res_ang = pitch_res_deg * 60
    res = f"Pitch angle resolution (angular minutes) of '{_laser_name(lidar_name)}' sensor: {pitch_res_ang:.2f}'"
    print(res)


### Exercise C1-3-1 : print no. of vehicles
def print_no_of_vehicles(
        frame: dataset_pb2.Frame, use_laser_counts=True
):
    """Prints the number of ground-truth vehicles in the frame.

    The object counts returned are obtained from the LiDAR sensor data.
    Recall `extract_frame_data` in `download_extract.py` from Course 1,
    see: https://bit.ly/ND001311de.
    
    :param frame: the Waymo Open Dataset `Frame` instance,
        here we use the simple dataset reader library to parse the frame.
    :param use_laser_counts: bool (optional), whether or not to use laser
        object counts. If False, camera object counts will be used.
    """

    print("Exercise C1-3-1")
    # find out the number of labeled vehicles in the given frame
    # Hint: inspect the data structure frame.laser_labels
    num_vehicles = 0
    if use_laser_counts:
        object_counts = frame.context.stats.laser_object_counts
    else:
        object_counts = frame.context.stats.camera_object_counts
    for x in object_counts:the 
        if _object_type_name(x.type) == 'TYPE_VEHICLE':
            num_vehicles = x.count
            break
        else:
            continue           
    print("Number of labelled vehicles in current frame:", num_vehicles)