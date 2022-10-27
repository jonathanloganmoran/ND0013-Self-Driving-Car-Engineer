# ---------------------------------------------------------------------
# Exercises from lesson 2 (object detection)
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.  
#
# Purpose of this file : Starter Code
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

from PIL import Image
import io
import sys
import os
import cv2
import open3d as o3d
import math
import numpy as np
import zlib

import matplotlib    
import matplotlib.pyplot as plt

# Exercise C2-4-6 : Plotting the precision-recall curve
def plot_precision_recall():
    pass
    # Please note: this function assumes that you have pre-computed the precions/recall value pairs from the test sequence
    #              by subsequently setting the variable configs.conf_thresh to the values 0.1 ... 0.9 and noted down the results.
    
    # Please create a 2d scatter plot of all precision/recall pairs 



# Exercise C2-3-4 : Compute precision and recall
def compute_precision_recall(
        det_performance_all: list, conf_thresh: float=0.5
):
    """Compute the precision and recall scores for a set of detections.

    :param det_performance_all: the nested `list` of metrics for each frame.
    :param conf_thresh: the confidence threshold used to compute the metrics.
    """

    if len(det_performance_all)==0 :
        print("no detections for conf_thresh = " + str(conf_thresh))
        return
    ### Extract the total number of positives, true positives, false negatives and false positives
    # The format of `det_performance_all` is `[ious, center_devs, pos_negs]`
    pos_negs_list = []
    for item in det_performance_all:
        # Get the `pos_negs` list for this image and append to list
        pos_negs_list.append(item[2])
    # Convert the list to a Numpy `ndarray` for better slicing
    pos_negs_list = np.asarray(pos_negs_list)
    ### Compute the metrics
    # The format of `pos_negs_list` entry is `[num_detections, true_positives, false_negatives, false_positives]` 
    num_detections = sum(pos_negs_list[:, 0])
    true_positives = sum(pos_negs_list[:, 1])
    false_negatives = sum(pos_negs_list[:, 2])
    false_positives = sum(pos_negs_list[:, 3])
    # Note that we do not compute true negatives for this task (assume all objects should be detected)
    print("TP = " + str(true_positives) + ", FP = " + str(false_positives) + ", FN = " + str(false_negatives))
    ### Compute precision
    precision = true_positives / (true_positives + false_positives)
    ### Compute recall
    recall = true_positives / (true_positives + false_negatives)
    print("precision = " + str(precision) + ", recall = " + str(recall) + ", conf_thres = " + str(conf_thresh) + "\n")    


### Exercise C2-3-2 : Transform metric point coordinates to BEV space
def pcl_to_bev(
        lidar_pcl: np.ndarray, configs: easydict.EasyDict, vis: bool=True
):
    """Converts the point cloud to a BEV-map of intensity values.

    :param lidar_pcl: the point cloud to clip and convert to BEV-map.
    :param configs: the EasyDict instance storing the viewing range and
        filtering params used to crop, discretise and re-scale the PCL values.
    :param vis: bool (optional), indicates whether or not to display the BEV-map.
    """

    ### Compute BEV-map discretisation by dividing x-range by the BEV-image height
    bev_interval = (configs.lim_x[1] - configs.lim_x[0]) / configs.bev_height
    ### Create a copy of the lidar pcl and transform all matrix x-coordinates into BEV-image coordinates    
    lidar_pcl_cpy = lidar_pcl.copy()
    lidar_pcl_cpy[:, 0] = np.int_(np.floor(lidar_pcl_cpy[:, 0] / bev_interval))
    ### Transform all matrix y-coordinates but centre the forward-facing x-axis on the middle of the image
    lidar_pcl_cpy[:, 1] = np.int_(np.floor(lidar_pcl_cpy[:, 1] / bev_interval) + (configs.bev_width + 1) / 2)
    ### Shift level of ground plane to avoid flipping from 0 to 255 for neighbouring pixels
    lidar_pcl_cpy[:, 2] = lidar_pcl_cpy[:, 2] - configs.lim_z[0]
    ### Re-arrange elements in `lidar_pcl_cpy` by sorting first by x, then y, then by decreasing height
    idxs_height = np.lexsort(keys=(-lidar_pcl_cpy[:, 2], lidar_pcl_cpy[:, 1], lidar_pcl_cpy[:, 0]))
    lidar_pcl_height = lidar_pcl_cpy[idxs_height]
    ### Extract all points with identical x and y s.t. only the top-most z-coordinate is kept
    _, idxs_height_unique = np.unique(lidar_pcl_height[:, 0:2], axis=0, return_index=True)
    lidar_pcl_height = lidar_pcl_height[idxs_height_unique]
    ### Assign the height value of each unique entry in `lidar_pcl_height` to the height map 
    height_map = np.zeros(shape=(configs.bev_height + 1, configs.bev_height + 1))
    # Make sure that each entry is normalized on the difference between the upper and lower BEV-map height
    height_map[np.int_(lidar_pcl_height[:, 0]), np.int_(lidar_pcl_height[:, 1])] = lidar_pcl_height[:, 2] / float(np.abs(configs.lim_z[1] - configs.lim_z[0]))
    ### Sort points s.t. in case of identical BEV grid coordinates, the points in each grid cell are arranged based on their intensity
    # Here we clip the intensity values to 1.0
    lidar_pcl_cpy[lidar_pcl_cpy[:, 3] > 1.0, 3] = 1.0
    idxs_intensity = np.lexsort(keys=(-lidar_pcl_cpy[:, 3], lidar_pcl_cpy[:, 1], lidar_pcl_cpy[:, 0]))
    lidar_pcl_cpy = lidar_pcl_cpy[idxs_intensity]
    ### Only keep one point per grid cell
    _, idxs_intensity = np.unique(lidar_pcl_cpy[:, 0:2], axis=0, return_index=True)
    lidar_intensity = lidar_pcl_cpy[idxs_intensity]
    ### Create the intensity map
    intensity_map = np.zeros(shape=(configs.bev_height + 1, configs.bev_height + 1))
    intensity_map[np.int_(lidar_intensity[:, 0]), np.int_(lidar_intensity[:, 1])] = lidar_intensity[:, 3] / (np.amax(lidar_intensity[:, 3]) - np.amin(lidar_intensity[:, 3]))
    ### Visualise intensity map
    if vis:
       img_intensity = intensity_map * 256
       img_intensity = img_intensity.astype(np.uint8)
       while (1):
           cv2.imshow('img_intensity_BEV', img_intensity)
           if cv2.waitKey(10) & 0xFF == 27:
               break
       cv2.destroyAllWindows()
