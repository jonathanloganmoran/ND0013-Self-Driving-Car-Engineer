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


def _object_type_name(x: int) -> str:
    """Returns the class label mapped to the input class id."""
    return label_pb2.Label.Type.Name(x)


# Exercise C1-5-5 : Visualize intensity channel
def vis_intensity_channel(frame, lidar_name):

    print("Exercise C1-5-5")
    # extract range image from frame

    # map value range to 8bit

    # focus on +/- 45° around the image center



# Exercise C1-5-2 : Compute pitch angle resolution
def print_pitch_resolution(frame, lidar_name):

    print("Exercise C1-5-2")
    # load range image
        
    # compute vertical field-of-view from lidar calibration 

    # compute pitch resolution and convert it to angular minutes


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
    for x in object_counts:
        if _object_type_name(x.type) == 'TYPE_VEHICLE':
            num_vehicles = x.count
            break
        else:
            continue           
    print("number of labeled vehicles in current frame:" + str(num_vehicles))