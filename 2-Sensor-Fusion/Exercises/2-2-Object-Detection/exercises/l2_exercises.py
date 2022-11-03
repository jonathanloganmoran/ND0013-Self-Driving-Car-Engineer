# ------------------------------------------------------------------------------
# Exercises from Lesson 2.2: Detecting Objects in LiDAR
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
import easydict
import io
import math
import matplotlib    
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from open3d import JVisualizer
import os
from PIL import Image
import sys
import zlib


# Exercise C2-4-6 : Plotting the precision-recall curve
def plot_precision_recall():
    # Please note: this function assumes that you have pre-computed the precision
    #              and recall value pairs from the test sequence by subsequently
    #              setting the variable `configs.conf_thresh` to the values
    #              0.1 ... 0.9 and noted down the results.
    # Please create a 2d scatter plot of all precision/recall pairs
    pass


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
    ### Extract the total number of positives, true positives, false negatives,
    #   and false positives. The format of `det_performance_all` is:
    #       `[ious, center_devs, pos_negs]`.
    pos_negs_list = []
    for item in det_performance_all:
        # Get the `pos_negs` list for this image and append to list
        pos_negs_list.append(item[2])
    # Convert the list to a Numpy `ndarray` for better slicing
    pos_negs_list = np.asarray(pos_negs_list)
    ### Compute the metrics
    # The format of `pos_negs_list` entry is:
    #     `[num_detections, true_positives, false_negatives, false_positives]` 
    num_detections = sum(pos_negs_list[:, 0])
    true_positives = sum(pos_negs_list[:, 1])
    false_negatives = sum(pos_negs_list[:, 2])
    false_positives = sum(pos_negs_list[:, 3])
    # Note: we do not compute true negatives for this task
    #       i.e., we assume that all ground-truth labels are objects to detect 
    print(f"TP: {true_positives}, FP: {false_positives}, FN: {false_negatives}")
    ### Compute precision
    precision = true_positives / (true_positives + false_positives)
    ### Compute recall
    recall = true_positives / (true_positives + false_negatives)
    res = f"Precision: {precision}, Recall: {recall}, conf_thres: {conf_thresh}"
    print(res)


### Exercise C2-3-2 : Transform metric point coordinates to BEV space
def pcl_to_bev(
        lidar_pcl: np.ndarray, configs: easydict.EasyDict,
        vis: bool=False, inline: bool=False
):
    """Converts the point cloud to a BEV-map of intensity values.

    :param lidar_pcl: the point cloud to clip and convert to BEV map.
    :param configs: the EasyDict instance storing the viewing range and
        filtering params used to crop, discretise and re-scale the PCL values.
    :param vis: bool (optional), indicates if the BEV map should be displayed.
    :param inline: bool (optional), If True, the BEV map will be rendered using
        Matplotlib `figure` instance. If False (and `vis=True`), OpenCV is used.
    """

    ### Compute BEV map discretisation
    # Here we divide the x-range by the BEV image height
    bev_interval = (configs.lim_x[1] - configs.lim_x[0]) / configs.bev_height
    bev_offset = (configs.bev_width + 1) / 2
    ### Transform all matrix x-coordinates into BEV image coordinates
    # Here we create a copy to avoid mutation of the original PCL
    lidar_pcl_cpy = lidar_pcl.copy()
    lidar_pcl_cpy[:, 0] = np.int_(np.floor(lidar_pcl_cpy[:, 0] / bev_interval))
    ### Transform all matrix y-coordinates
    # Here we centre the forward-facing x-axis on the middle of the image
    lidar_pcl_cpy[:, 1] = np.int_(
        np.floor(lidar_pcl_cpy[:, 1] / bev_interval) + bev_offset
    )
    ### Shift ground plane to avoid flipping neighbouring pixels from 0 to 255
    lidar_pcl_cpy[:, 2] = lidar_pcl_cpy[:, 2] - configs.lim_z[0]
    ### Re-arrange `lidar_pcl_cpy` values
    # Here we first sort by x, then y, then by decreasing height values
    idxs_intensity = np.lexsort(
        keys=(-lidar_pcl_cpy[:, 2], lidar_pcl_cpy[:, 1], lidar_pcl_cpy[:, 0])
    )
    lidar_pcl_top = lidar_pcl_cpy[idxs_intensity]
    ### Extract all points with identical x and y,
    #   s.t. only the top-most z-coordinate is kept
    _, idxs_top_unique = np.unique(
        lidar_pcl_top[:, 0:2], axis=0, return_index=True
    )
    lidar_pcl_top = lidar_pcl_top[idxs_top_unique]
    ### Assign height map the elongation value of each unique entry 
    height_map = np.zeros(shape=(configs.bev_height, configs.bev_width))
    # Here we perform min-max normalisation to mitigate influence of outliers 
    scale_factor_height = float(np.abs(configs.lim_z[1] - configs.lim_z[0]))
    height_map[np.int_(lidar_pcl_top[:, 0]), 
               np.int_(lidar_pcl_top[:, 1])
              ] = lidar_pcl_top[:, 2] / scale_factor_height
    ### Sort identical BEV grid coordinate points
    # Here we clip the intensity values to 1.0
    lidar_pcl_cpy[lidar_pcl_cpy[:, 3] > 1.0, 3] = 1.0
    # Here we arrange grid cell values based on intensity
    idxs_intensity = np.lexsort(
        keys=(-lidar_pcl_cpy[:, 3], lidar_pcl_cpy[:, 1], lidar_pcl_cpy[:, 0])
    )
    lidar_pcl_int = lidar_pcl_cpy[idxs_intensity]
    ### Only keep one point per grid cell
    _, idxs_intensity = np.unique(
        lidar_pcl_cpy[:, 0:2], axis=0, return_index=True
    )
    lidar_pcl_int = lidar_pcl_cpy[idxs_intensity]
    ### Create the intensity map
    intensity_map = np.zeros(
        shape=(configs.bev_height + 1, configs.bev_height + 1)
    )
    scale_factor_intensity = (
        np.amax(lidar_pcl_int[:, 3]) - np.amin(lidar_pcl_int[:, 3])
    )
    intensity_map[np.int_(lidar_pcl_int[:, 0]),
                  np.int_(lidar_pcl_int[:, 1])
                 ] = lidar_pcl_int[:, 3] / scale_factor_intensity
    ### Visualise intensity map
    if vis:
        img_intensity = intensity_map * 256
        img_intensity = img_intensity.astype(np.uint8)
        str_title = "Bird's-eye view (BEV) height map: intensity channel"
        if inline:
            fig = plt.figure(figsize=(24, 20))
            plt.title(str_title, fontsize=20)
            plt.imshow(img_intensity)
        else:
            while (1):
                cv2.imshow(str_title, img_intensity)
                if cv2.waitKey(10) & 0xFF == 27:
                    break
            cv2.destroyAllWindows()
    else:
        return