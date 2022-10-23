# ---------------------------------------------------------------------
# Exercises from lesson 1 (lidar)
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.  
#
# Purpose of this file : Starter Code
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

import cv2
import io
import numpy as np
import os
from PIL import Image
import sys
import zlib

### Add current working directory to path
sys.path.append(os.getcwd())

### Waymo Open Dataset Reader library
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2
from tools.waymo_reader.simple_waymo_open_dataset_reader import label_pb2
from l1_examples import load_range_image


def _object_type_name(x: int) -> str:
    """Returns the class label mapped to the input class id."""
    return label_pb2.Label.Type.Name(x)


# Exercise C1-5-5 : Visualize intensity channel
def vis_intensity_channel(frame, lidar_name):
    """Display the centre-front segment of range image.

    The range image is converted to 8-bit grayscale and
    the centre-front ROI is obtained with a +/- 45deg offset
    window from the x-axis at the origin.

    We assume that the LiDAR sensor has been calibrated
    exactly to the direction of motion of the ego-vehicle,
    i.e., that the x-axis origin is exactly in this direction,
    and that the front-half of the range image consistutes a
    180deg segment, such that a -/+ 45deg shift from the centre
    x-axis makes up a 90deg 'front-facing' ROI.

    :param frame: the Waymo Open Dataset `Frame` instance.
    :param lidar_name: the class id mapped to the LiDAR sensor to fetch.
    """

    print("Exercise C1-5-5")
    ### Extract the range image from the frame
    ri = load_range_image(frame, lidar_name)
    ### Map the value range to 8-bit grayscale
    img_range = ri[:, :, 0]
    img_range = img_range * 255 / (np.amax(img_range) - np.amin(img_range))
    img_range = img_range.astype(np.uint8)
    ### Focus image on +/- 45° ROI about the centre along the x-axis (see assumptions)
    deg45 = int(img_range.shape[1] / 8)
    ri_centre = int(img_range.shape[1] / 2)
    img_range = img_range[:, ri_centre - deg45:ri_centre + deg45]
    ### Visualising the resulting cropped 8-bit range image
    cv2.imshow('range_image', img_range)
    cv2.waitKey(0)


# Exercise C1-5-2 : Compute pitch angle resolution
def print_pitch_resolution(frame: dataset_pb2.Frame, lidar_name: int):
    """Prints the vertical angular resoution of the range image.

    Using the equally-divided elevation angle formula from the 
    Projection By Elevation Angle (PBEA) method, such that:
    $$\begin{align}
        \theta_{res} = (\theta_{up} - \theta_{down}) / h, \\
    \end{align}$$
    for maximum and minimum elevation angles $\theta_{up}$ and $\theta{down}$,
    respectively, and for height $h$ of the range image.

    :param frame: the Waymo Open Dataset `Frame` instance.
    :param lidar_name: the class id mapped to the LiDAR sensor to fetch.
    """

    print("Exercise C1-5-2")
    ### Load the range image
    ri = load_range_image(frame, lidar_name)
    ### Compute vertical field-of-view (VFOV) from lidar calibration 
    # Get the laser calibration data
    laser_calib = [sensor for sensor in frame.context.laser_calibrations if sensor.name == lidar_name][0] 
    # Get the maximum and minimum pitch elevation angles
    pitch_max = lidar_calib.beam_inclination_max
    pitch_min = lidar_calib.beam_inclination_min
    # Compute the vertical resolution in angular degrees
    vfov = pitch_max - pitch_min
    pitch_res_rad = vfov / ri.shape[0]
    pitch_res_deg = pitch_res_rad * 180 / np.pi
    # print(f'Pitch angle resolution: {pitch_res_deg:.2f}°')
    # Convert to angular minutes
    print(f"Pitch angle resolution: {(pitch_res_deg * 60):.2f}'")


### Exercise C1-3-1 : print no. of vehicles
def print_no_of_vehicles(frame: dataset_pb2.Frame, use_laser_counts=True):
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
    print("number of labeled vehicles in current frame:" + str(num_vehicles))