# ------------------------------------------------------------------------------
# Project "3D Object Detection with LiDAR Data"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Evaluate performance of object detection
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
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from operator import itemgetter
import os
from shapely.geometry import Polygon
import sys
import torch
from typing import List

# Change backend so that figure maximizing works on Mac as well
matplotlib.use('wxagg')

### Add project directory to PYTHONPATH to enable relative imports
# Alternatively, use the `pip install ..` script with setuptools
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(
    os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__)))
)
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

### Object detection tools and helper functions
import misc.objdet_tools as tools


### Compute various performance measures to assess object detection (ID_S4_EX1)
def measure_detection_performance(
        detections: List[list], labels: List[list], labels_valid: List[list],
        min_iou: float=0.5
):
    """Returns the computed detection performance measures.

    :param detections:
    :param labels:
    :param labels_valid:
    :param min_iou:
    :returns: det_performance,
    """
    
    ### Find the best detection for each valid label
    true_positives = 0    # Num. correctly detected objects
    center_devs = []
    ious = []
    # 
    for label, valid in zip(labels, labels_valid):
        matches_lab_det = []
        # Exclude all invalid labels from metric computations
        if valid:
            ####### ID_S4_EX1 START #######     
            print("student task ID_S4_EX1 ")
            ### Calculate the Intersection over Union (IoU) and
            # the distance between bounding box centres
            ### Step 1 : Extract the four coordinate pairs of the bounding box
            box = label.box
            box_1 = tools.compute_box_corners(
                        box.center_x,
                        box.center_y,
                        box.width,
                        box.length,
                        box.heading
            )
            ### Step 2 : Loop over all detected objects
            for det in detections:
                ### Step 3 : Extract the four corners of the current detection
                _id, x, y, z, _h, w, l, yaw = det
                box_2 = tools.compute_box_corners(x, y, z, w, l, yaw)
                ### Step 4 : Computer the distance between two bounding boxes
                # Distance is measured from the centre of the ground-truth to
                # the centre of the predicted bounding box in (x, y, z)
                dist_x = np.array(box.center_x - x).item()
                dist_y = np.array(box.center_y - y).item()
                dist_z = np.array(box.center_z - z).item()
                ### Step 5 : Compute the Intersection over Union (IOU) between
                # the ground-truth and the predicted bounding boxes
                try:
                    poly_1 = Polygon(box_1)
                    poly_2 = Polygon(box_2)
                    intersection = poly_1.intersection(poly_2).area
                    union = poly_1.union(poly_2).area
                    iou = intersection / union
                except Exception as err:
                    print(f"Encountered '{err}' error in IoU calculation")
                ### Step 6 : Evaluate IoU results
                # We store `[iou, dist_x, dist_y, dist_z]` in `matched_lab_det`
                # and increase the TP count if the IoU exceeds `min_iou`
                if iou > min_iou:
                    matches_lab_det.append([iou, dist_x, dist_y, dist_z])
                    true_positives += 1
            ####### ID_S4_EX1 END #######
        ### Find the best bounding box match and compute metrics
        # Here we select the detection with the greatest IoU score
        if matches_lab_det:
            # Retrieve entry with max IoU score in case of multiple candidates
            best_match = max(matches_lab_det, key=itemgetter(1))  
            ious.append(best_match[0])
            center_devs.append(best_match[1:])
    ####### ID_S4_EX2 START #######
    print("student task ID_S4_EX2")
    ### Compute the metrics for precision / recall score
    ### Step 1 : Compute the total number of positives present in the scene
    # Here we have the total number of possible objects to detect
    all_positives = labels_valid.sum()
    # Here we count only the best predictions from the total number of correctly
    # predicted objects, i.e., matched predictions with IoU above threshold
    true_positives = len(ious)
    ### Step 2 : Compute the number of false negatives
    false_negatives = all_positives - true_positives
    ### step 3 : compute the number of false positives
    false_positives = len(detections) - true_positives
    ####### ID_S4_EX2 END #######     
    
    pos_negs = [all_positives, true_positives, false_negatives, false_positives]
    det_performance = [ious, center_devs, pos_negs]
    
    return det_performance


# evaluate object detection performance based on all frames
def compute_performance_stats(det_performance_all):

    # extract elements
    ious = []
    center_devs = []
    pos_negs = []
    for item in det_performance_all:
        ious.append(item[0])
        center_devs.append(item[1])
        pos_negs.append(item[2])
    
    ####### ID_S4_EX3 START #######     
    #######    
    print('student task ID_S4_EX3')

    ## step 1 : extract the total number of positives, true positives, false negatives and false positives
    
    ## step 2 : compute precision
    precision = 0.0

    ## step 3 : compute recall 
    recall = 0.0

    #######    
    ####### ID_S4_EX3 END #######     
    print('precision = ' + str(precision) + ", recall = " + str(recall))   

    # serialize intersection-over-union and deviations in x,y,z
    ious_all = [element for tupl in ious for element in tupl]
    devs_x_all = []
    devs_y_all = []
    devs_z_all = []
    for tuple in center_devs:
        for elem in tuple:
            dev_x, dev_y, dev_z = elem
            devs_x_all.append(dev_x)
            devs_y_all.append(dev_y)
            devs_z_all.append(dev_z)
    

    # compute statistics
    stdev__ious = np.std(ious_all)
    mean__ious = np.mean(ious_all)

    stdev__devx = np.std(devs_x_all)
    mean__devx = np.mean(devs_x_all)

    stdev__devy = np.std(devs_y_all)
    mean__devy = np.mean(devs_y_all)

    stdev__devz = np.std(devs_z_all)
    mean__devz = np.mean(devs_z_all)
    #std_dev_x = np.std(devs_x)

    # plot results
    data = [precision, recall, ious_all, devs_x_all, devs_y_all, devs_z_all]
    titles = ['detection precision', 'detection recall', 'intersection over union', 'position errors in X', 'position errors in Y', 'position error in Z']
    textboxes = ['', '', '',
                 '\n'.join((r'$\mathrm{mean}=%.4f$' % (np.mean(devs_x_all), ), r'$\mathrm{sigma}=%.4f$' % (np.std(devs_x_all), ), r'$\mathrm{n}=%.0f$' % (len(devs_x_all), ))),
                 '\n'.join((r'$\mathrm{mean}=%.4f$' % (np.mean(devs_y_all), ), r'$\mathrm{sigma}=%.4f$' % (np.std(devs_y_all), ), r'$\mathrm{n}=%.0f$' % (len(devs_x_all), ))),
                 '\n'.join((r'$\mathrm{mean}=%.4f$' % (np.mean(devs_z_all), ), r'$\mathrm{sigma}=%.4f$' % (np.std(devs_z_all), ), r'$\mathrm{n}=%.0f$' % (len(devs_x_all), )))]

    f, a = plt.subplots(2, 3)
    a = a.ravel()
    num_bins = 20
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    for idx, ax in enumerate(a):
        ax.hist(data[idx], num_bins)
        ax.set_title(titles[idx])
        if textboxes[idx]:
            ax.text(0.05, 0.95, textboxes[idx], transform=ax.transAxes, fontsize=10,
                    verticalalignment='top', bbox=props)
    plt.tight_layout()
    plt.show()

