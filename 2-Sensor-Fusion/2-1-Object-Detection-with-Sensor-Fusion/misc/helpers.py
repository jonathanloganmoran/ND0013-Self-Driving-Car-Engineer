# ------------------------------------------------------------------------------
# Project "3D Object Detection with LiDAR Data"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Loop over all frames in a Waymo Open Dataset file,
#                        detect and track objects and visualize results.
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
import os
import pickle
from typing import List


### List of all valid executions to select from
# Data manipulation functions
DATA = [
    'pcl_from_rangeimage', 'load_image'
]
# Object detection functions
DETECTION = [
    'bev_from_pcl', 'detect_objects',
    'validate_object_labels',
    'measure_detection_performance'
]
# Multi-target tracking (MTT) functions
TRACKING = [
    'perform_tracking'
]
# BEV map visualisation functions
VISUALIZATION = [
    'show_range_image', 'show_bev', 'show_pcl',
    'show_labels_in_image', 'show_objects_and_labels_in_bev',
    'show_objects_in_bev_labels_in_camera', 'show_tracks',
    'show_detection_performance', 'make_tracking_movie'
]


## Saves an object to a binary file
def save_object_to_file(object, file_path, base_filename, object_name, frame_id=1):
    object_filename = os.path.join(file_path, os.path.splitext(base_filename)[0]
                                   + "__frame-" + str(frame_id) + "__" + object_name + ".pkl")
    with open(object_filename, 'wb') as f:
        pickle.dump(object, f)

## Loads an object from a binary file
def load_object_from_file(file_path, base_filename, object_name, frame_id=1):
    object_filename = os.path.join(file_path, os.path.splitext(base_filename)[0]
                                   + "__frame-" + str(frame_id) + "__" + object_name + ".pkl")
    with open(object_filename, 'rb') as f:
        object = pickle.load(f)
        return object


### Prepares an `exec_list` with all tasks to be executed
def make_exec_list(
        exec_detection: List[str], exec_tracking: List[str],
        exec_visualization: List[str], exec_data: List[str]=None
) -> List[str]:
    """Prepares a list programme executions to run.

    :param exec_detection: list of desired object detection functions to run.
    :param exec_tracking: list of desired tracking functions to run.
    :param exec_visualization: list of desired visualization functions to run.
    :param exec_data: list of desired data manipulation functions to run.
    :returns: exec_list, global execution list to run.
    """
    
    ### Remove any unsupported functions from input lists
    exec_data = list(set(DATA).intersection(set(exec_data)))
    exec_detection = list(set(DETECTION).intersection(set(exec_detection)))
    exec_tracking = list(set(TRACKING).intersection(set(exec_tracking)))
    exec_visualization = list(set(VISUALIZATION).intersection(
                                    set(exec_visualization)))
    ### Save all tasks in `exec_list`
    exec_list = exec_data + exec_detection + exec_tracking + exec_visualization
    ### Check if we need any point cloud dependency functions
    if any(i in exec_list for i in ('validate_object_labels', 'bev_from_pcl')):
        if 'pcl_from_rangeimage' in exec_list:
            # Append function as a dependency
            exec_list.append('pcl_from_rangeimage')
    ### Check if we are running any image manipulation functions
    if any(i in exec_list for i in ('show_tracks', 'show_labels_in_image', 'show_objects_in_bev_labels_in_camera')):
        if 'load_image' not in exec_list:
            # Append function as a dependency
            exec_list.append('load_image')
    ### Check if we are running output movie function
    if 'make_tracking_movie' in exec_list:
        if 'show_tracks' not in exec_list:
            # Append function as a dependency
            exec_list.append('show_tracks')
    return exec_list