# ------------------------------------------------------------------------------
# Project "3D Object Detection with LiDAR Data"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Process and prepare point cloud for object detection.
#
# You should have received a copy of the Udacity license with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
#
# NOTE: The current version of this programme relies on Numpy to perform data 
#       manipulation, however, a platform-specific implementation, e.g.,
#       TensorFlow `tf.Tensor` data ops, is recommended.
# -----------------------------------------------------------------------------

### General package imports
import cv2
import numpy as np
import torch

### Add project directory to PYTHONPATH to enable relative imports
# Alternatively, use the `pip install ..` script with setuptools
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

### Simple Waymo Open Dataset Reader library
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2, label_pb2

### Object detection tools and helper functions
import misc.objdet_tools as tools


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
# See `../Exercises/2-1-Lidar-Sensor/examples/l1_examples.py`
def _load_range_image(
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


# visualize lidar point-cloud
def show_pcl(pcl):

    ####### ID_S1_EX2 START #######     
    #######
    print("student task ID_S1_EX2")

    # step 1 : initialize open3d with key callback and create window
    
    # step 2 : create instance of open3d point-cloud class

    # step 3 : set points in pcd instance by converting the point-cloud into 3d vectors (using open3d function Vector3dVector)

    # step 4 : for the first frame, add the pcd instance to visualization using add_geometry; for all other frames, use update_geometry instead
    
    # step 5 : visualize point cloud and keep window open until right-arrow is pressed (key-code 262)

    #######
    ####### ID_S1_EX2 END #######     
       

### Visualise range image (ID_S1_EX1)
def show_range_image(
        frame: dataset_pb2.Frame, lidar_name: int
) -> np.ndarray:
    """Returns the range image given in the `frame` captured by `lidar_name` sensor.

    The range and intensity channels of the range image are stacked and
    returned after re-scaling and normalisation is performed.

    :param frame: the Waymo Open Dataset `Frame` instance.
    :param lidar_name: the integer id corresponding to the
        LiDAR sensor name from which to extract and convert the range image.
    :returns: img_range_intensity, the range and intensity channels of the
        range image as a Numpy `ndarray` object. To be later visualised with OpenCV.
    """

    ####### ID_S1_EX1 START #######     
    print("student task ID_S1_EX1")
    ### Step 1 : Extract LiDAR data and range image for roof-mounted LiDAR sensor
    ri = _load_range_image(frame, lidar_name)
    ### Step 2 : Extract the range and the intensity channel from the range image
    ri_range = ri[:, :, 0]
    ri_intensity = ri[:, :, 1]
    ### Step 3 : Clip the range values to reasonable limits
    # Here we set invalid values (i.e., 'no return' values < 0) to zero
    MIN_RANGE = 0
    # Then clip the max range values to the limits defined in original paper
    MAX_RANGE = 75 if lidar_name is dataset_pb2.LaserName.TOP else 20  
    np.clip(ri_range, MIN_RANGE, MAX_RANGE)
    ### Step 4 : Map the range channel onto an 8-bit scale
    # Here we perform min-max normalisation and re-scale to 8-bit range
    ri_range = ri_range * 255 / (np.amax(ri_range) - np.amin(ri_range))
    ri_range = ri_range.astype(np.uint8)
    ### Step 5 : Map the intensity channel onto an 8-bit scale
    # Here we perform a contrast adjustment normalisation using the 1- and 99-percentile
    ri_inten_scaled = ri_intensity * np.amax(ri_intensity) / 2
    ri_intensity = ri_inten_scaled * 255 / (np.amax(ri_intensity) - np.amin(ri_intensity))
    ri_intensity = ri_intensity.astype(np.uint8)
    ### Step 6 : Stack the range and intensity image vertically using np.vstack
    # Then convert the result to unsigned 8-bit integer type
    img_range_intensity = np.vstack((ri_range, ri_intensity))
    img_range_intensity = img_range_intensity.astype(np.uint8)
    # Return resulting range image to be visualised with OpenCV inside `loop_over_dataset.py` 
    ####### ID_S1_EX1 END #######
    return img_range_intensity


# create birds-eye view of lidar data
def bev_from_pcl(lidar_pcl, configs):

    # remove lidar points outside detection area and with too low reflectivity
    mask = np.where((lidar_pcl[:, 0] >= configs.lim_x[0]) & (lidar_pcl[:, 0] <= configs.lim_x[1]) &
                    (lidar_pcl[:, 1] >= configs.lim_y[0]) & (lidar_pcl[:, 1] <= configs.lim_y[1]) &
                    (lidar_pcl[:, 2] >= configs.lim_z[0]) & (lidar_pcl[:, 2] <= configs.lim_z[1]))
    lidar_pcl = lidar_pcl[mask]
    
    # shift level of ground plane to avoid flipping from 0 to 255 for neighboring pixels
    lidar_pcl[:, 2] = lidar_pcl[:, 2] - configs.lim_z[0]  

    # convert sensor coordinates to bev-map coordinates (center is bottom-middle)
    ####### ID_S2_EX1 START #######     
    #######
    print("student task ID_S2_EX1")

    ## step 1 :  compute bev-map discretization by dividing x-range by the bev-image height (see configs)

    ## step 2 : create a copy of the lidar pcl and transform all metrix x-coordinates into bev-image coordinates    

    # step 3 : perform the same operation as in step 2 for the y-coordinates but make sure that no negative bev-coordinates occur

    # step 4 : visualize point-cloud using the function show_pcl from a previous task
    
    #######
    ####### ID_S2_EX1 END #######     
    
    
    # Compute intensity layer of the BEV map
    ####### ID_S2_EX2 START #######     
    #######
    print("student task ID_S2_EX2")

    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map

    # step 2 : re-arrange elements in lidar_pcl_cpy by sorting first by x, then y, then -z (use numpy.lexsort)

    ## step 3 : extract all points with identical x and y such that only the top-most z-coordinate is kept (use numpy.unique)
    ##          also, store the number of points per x,y-cell in a variable named "counts" for use in the next task

    ## step 4 : assign the intensity value of each unique entry in lidar_top_pcl to the intensity map 
    ##          make sure that the intensity is scaled in such a way that objects of interest (e.g. vehicles) are clearly visible    
    ##          also, make sure that the influence of outliers is mitigated by normalizing intensity on the difference between the max. and min. value within the point cloud

    ## step 5 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background

    #######
    ####### ID_S2_EX2 END ####### 


    # Compute height layer of the BEV map
    ####### ID_S2_EX3 START #######     
    #######
    print("student task ID_S2_EX3")

    ## step 1 : create a numpy array filled with zeros which has the same dimensions as the BEV map

    ## step 2 : assign the height value of each unique entry in lidar_top_pcl to the height map 
    ##          make sure that each entry is normalized on the difference between the upper and lower height defined in the config file
    ##          use the lidar_pcl_top data structure from the previous task to access the pixels of the height_map

    ## step 3 : temporarily visualize the intensity map using OpenCV to make sure that vehicles separate well from the background

    #######
    ####### ID_S2_EX3 END #######       

    # TODO remove after implementing all of the above steps
    lidar_pcl_cpy = []
    lidar_pcl_top = []
    height_map = []
    intensity_map = []

    # Compute density layer of the BEV map
    density_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    _, _, counts = np.unique(lidar_pcl_cpy[:, 0:2], axis=0, return_index=True, return_counts=True)
    normalizedCounts = np.minimum(1.0, np.log(counts + 1) / np.log(64)) 
    density_map[np.int_(lidar_pcl_top[:, 0]), np.int_(lidar_pcl_top[:, 1])] = normalizedCounts
        
    # assemble 3-channel bev-map from individual maps
    bev_map = np.zeros((3, configs.bev_height, configs.bev_width))
    bev_map[2, :, :] = density_map[:configs.bev_height, :configs.bev_width]  # r_map
    bev_map[1, :, :] = height_map[:configs.bev_height, :configs.bev_width]  # g_map
    bev_map[0, :, :] = intensity_map[:configs.bev_height, :configs.bev_width]  # b_map

    # expand dimension of bev_map before converting into a tensor
    s1, s2, s3 = bev_map.shape
    bev_maps = np.zeros((1, s1, s2, s3))
    bev_maps[0] = bev_map

    bev_maps = torch.from_numpy(bev_maps)  # create tensor from birds-eye view
    input_bev_maps = bev_maps.to(configs.device, non_blocking=True).float()
    return input_bev_maps


