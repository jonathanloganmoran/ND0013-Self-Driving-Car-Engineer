# ------------------------------------------------------------------------------
# Exercises from Lesson 2.2: Detecting Objects in LiDAR
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
from easydict import EasyDict
import io
import math
import numpy as np
import open3d as o3d
from open3d import JVisualizer
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

### Module to render / project bounding box labels
import misc.objdet_tools as tools


# Example C2-4-3 : Display detected objects on top of BEV map
def render_obj_over_bev(
        detections, lidar_bev_labels, configs, vis=False, inline=False
):
    """Renders the detected bounding box projections over the input BEV map.

    :param detections: the predicted bounding boxes to render over the BEV map.
    :param lidar_bev_labels: the ground-truth bounding boxes to render over the
        BEV map, assumed to be object returned from `render_obj_over_bev`.
    :param configs: the `EasyDict` instance with the desired BEV map dimensions.
    :param vis: bool (optional), If True, the BEV map will be displayed.
    :param inline: bool (optional), If True, the visualisation will be shown in
        Matplotlib `figure` instance. If False (and `vis=True`), the BEV map is
        rendered in a pop-up window using the `cv2.imshow` function.
    """

    ### Project the detected objects into Birds's-Eye View (BEV) map image
    tools.project_detections_into_bev(
        lidar_bev_labels, detections, configs, [0, 0, 255])
    ### Display the resulting BEV map
    if vis:
        lidar_bev_labels = cv2.rotate(lidar_bev_labels, cv2.ROTATE_180)
        str_title = "Bird's-eye view (BEV) map: ground-truth annotations shown with corresponding bounding box predictions."
        if inline:
            fig = plt.figure(figsize=(24, 20))
            plt.title(str_title, fontsize=20)
            plt.imshow(lidar_bev_labels)
        else:
            cv2.imshow(str_title, lidar_bev_labels)
            cv2.waitKey(0) 


# Example C2-4-3 : Display label bounding boxes on top of bev map
def render_bb_over_bev(
        bev_map, labels, configs, vis=False, inline=False
) -> np.ndarray:
    """Renders the bounding box labels over the input BEV map.

    :param bev_map: the BEV map representation of the frame.
    :param labels: the bounding box annotations to render over the BEV map.
    :param configs: the `EasyDict` instance with the desired BEV map dimensions.
    :param vis: bool (optional), If True, the BEV map will be displayed.
    :param inline: bool (optional), If True, the visualisation will be shown in
        Matplotlib `figure` instance. If False (and `vis=True`), the BEV map is
        rendered in a pop-up window using the `cv2.imshow` function.
    :returns: bev_map_cpy, the BEV map with annotations as a Numpy `ndarray` object. 
    """

    ### Convert BEV map from a tensor to Numpy array
    bev_map_cpy = bev_map.squeeze().permute(1, 2, 0).numpy() * 255
    bev_map_cpy = bev_map_cpy.astype(np.uint8)
    ### Resize the BEV map to desired dimensions
    bev_map_cpy = cv2.resize(bev_map_cpy, 
                             dsize=(configs.bev_width, configs.bev_height)
    )
    ### Convert the labels into proper bounding box format and project the
    #   bounding box coordinates into the BEV map image space
    label_objects = tools.convert_labels_into_objects(labels, configs)
    tools.project_detections_into_bev(
        bev_map_cpy, label_objects, configs, [0, 255, 0]
    )
    ### Display the resulting BEV map
    if vis:
        title_fig = "Bird's-eye view (BEV) map: shown with corresponding bounding box annotations"
        if inline:
            fig = plt.figure(figsize=(24, 20))
            plt.title(title_fig, fontsize=20)
            plt.imshow(bev_map_cpy)
        else:
            bev_map_cpy = cv2.rotate(bev_map_cpy, cv2.ROTATE_180)   
            cv2.imshow(title_fig, bev_map_cpy)
            cv2.waitKey(0)          
    return np.asarray(bev_map_cpy)


# Example C2-4-2 : count total no. of vehicles / vehicles difficult to track
def count_vehicles(
        frame: dataset_pb2.Frame
):
    """Prints number of ground-truth labels for the vehicle object class.

    We assume that all 'vehicle' objects belong to the 'TYPE_VEHICLE' class,
    and that the number of ground-truth labels correspond to the number of observable
    'TYPE_VEHICLE' objects in-frame and viewable by the respective laser sensor.

    :param frame: the Waymo Open Dataset `Frame` instance.
    """

    ### Initialise the static counter variables
    if not hasattr(count_vehicles, "cnt_vehicles"):
        count_vehicles.cnt_vehicles = 0
        count_vehicles.cnt_difficult_vehicles = 0
    ### Loop over all labels recorded for each LiDAR sensor
    for label in frame.laser_labels:
        # Note that we only record 'TYPE_VEHICLE' class annotations
        if label.type == label_pb2.Label.Type.TYPE_VEHICLE:
            # Here we add all label instances
            # even if they could be associated with the same object
            count_vehicles.cnt_vehicles += 1
            # We also make note of the number of level 'difficult' detections
            if label.detection_difficulty_level > 0:
                count_vehicles.cnt_difficult_vehicles += 1
    print(f"No. of labelled vehicles: {count_vehicles.cnt_vehicles}, No. of vehicles 'difficult' to detect {count_vehicles.cnt_difficult_vehicles}")


# Example C2-3-3 : Minimum and maximum intensity
def min_max_intensity(
        lidar_pcl: np.ndarray
):
    """Prints the minimum and maximum intensity values from the point cloud.

    :param lidar_pcl: the 3D point cloud instance as a Numpy `ndarray` object,
        Here we assume that the intensity channel is the last (fourth) column. 
    """

    ### Retrieve the minimum and maximum intensity values from the point cloud
    # Here we consider only the BEV intensity channel values
    min_int = np.amin(lidar_pcl[:, 3])
    max_int = np.amax(lidar_pcl[:, 3])
    print(f"Min. intensity value: {min_int}, Max. intensity value: {max_int}")


# Example C2-3-1 : Crop point cloud
def crop_pcl(
        lidar_pcl: np.ndarray, configs: easydict.EasyDict,
        vis: bool=False, inline: bool=False
) -> np.ndarray:
    """Crops the LiDAR point cloud to the ROI defined in `configs`.

    :param lidar_pcl: the 3D point cloud instance as a Numpy `ndarray` object.
    :param configs: the `EasyDict` instance storing the (x, y, z) coordinate
        limits of the desired point cloud region of interest (ROI).
    :param vis: bool (optional), If True, the cropped point cloud will be
        displayed in a window using the Open3D `Visualizer`.
    :param inline: bool (optional), If True, the visualisation will be displayed
        using the Open3D Jupyter notebook `JVisualizer` viewer. If False,
        (and `vis=True`), the point cloud is rendered in a pop-up window using
        the `o3d.visualization.draw_geometries` function.
    :returns: lidar_pcl, the cropped point cloud as a Numpy `ndarray` object.
    """

    pcl_range = lidar_pcl[:, 0]
    pcl_intensity = lidar_pcl[:, 1]
    pcl_elongation = lidar_pcl[:, 2]
    ### Create mask to remove points outside of pre-defined region of interest
    mask = np.where(
        (pcl_range >= configs.lim_x[0]) & (pcl_range <= configs.lim_x[1]) &
        (pcl_intensity >= configs.lim_y[0]) & (pcl_intensity <= configs.lim_y[1]) &
        (pcl_elongation >= configs.lim_z[0]) & (pcl_elongation <= configs.lim_z[1])
    )
    ### Preserve only the coordinates inside the ROI
    lidar_pcl = lidar_pcl[mask]
    ### Visualise the resulting cropped point-cloud
    if vis:
        pcd = o3d.geometry.PointCloud()
        # Here we select only the first three channels (excluding BEV intensity)
        # i.e., we omit the mapped BEV intensity channel values
        pcd.points = o3d.utility.Vector3dVector(lidar_pcl[:, 0:3])
        if inline:
            # Render the point cloud using Jupyter notebook viewer
            visualiser = JVisualizer()
            visualiser.add_geometry(pcd)
            visualiser.show()
        else:
            o3d.visualization.draw_geometries([pcd])
    return lidar_pcl
