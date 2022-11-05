# -------------------------------------------------------------------------------
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
# ------------------------------------------------------------------------------

### General package imports
import cv2
import easydict
import numpy as np
import open3d as o3d
import torch

### Add project directory to PYTHONPATH to enable relative imports
# Alternatively, use the `pip install ..` script with setuptools
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(
    os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__)))
)
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

### Simple Waymo Open Dataset Reader library
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2
from tools.waymo_reader.simple_waymo_open_dataset_reader import label_pb2

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
    """Returns the range image from the `frame` captured by `lidar_name` sensor.

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
        ri.ParseFromString(
            zlib.decompress(laser.ri_return1.range_image_compressed)
        )
        ri = np.array(ri.data).reshape(ri.shape.dims)
    return ri


### Visualise LiDAR point cloud (ID_S1_EX2)
def show_pcl(
        pcl: np.ndarray
):
    """Displays the LiDAR point cloud data in a Open3D viewer.

    Creates an Open3D `Visualizer` instance with custom key callback
    capability. To advance the frame, i.e., render the next point cloud,
    use the right-arrow key (key code 262).

    :param pcl: the 3D point cloud to visualise.
    """

    def close_window(vis: o3d.visualization.Visualizer) -> bool:
        # Notify the window to be closed
        vis.close()
        # Return boolean indicating if `update_geometry` needs to be run
        return False

    ####### ID_S1_EX2 START #######
    print("student task ID_S1_EX2")
    ### Step 1 : Initialise Open3D with key callback and create window
    visualiser = o3d.visualization.VisualizerWithKeyCallback()
    str_window = 'Visualising the Waymo Open Dataset: LiDAR Point Cloud data'
    visualiser.create_window(window_name=str_window,
                      width=1280, height=720, left=50, top=50, visible=True
    )
    # Here we register the callback function to trigger on right-arrow key press
    visualiser.register_key_callback(key=262, callback_func=close_window)
    ### Step 2 : Create instance of Open3D `PointCloud` class
    pcd = o3d.geometry.PointCloud()
    ### Step 3 : Set points in `pcd` instance using Open3D `Vector3dVector`
    # Here we convert the point cloud into 3D vectors
    pcd.points = o3d.utility.Vector3dVector(pcl[:, :3])
    ### Step 4 : Add / update the point cloud with the frame data
    # Here we use `add_geometry` for the first frame,
    # note that `update_geometry` is automatically used for all other frames
    visualiser.add_geometry(pcd)
    ### Step 5 : Visualise point cloud and preserve window instance
    visualiser.run()
    ####### ID_S1_EX2 END #######     
       

### Visualise range image (ID_S1_EX1)
def show_range_image(
        frame: dataset_pb2.Frame, lidar_name: int
) -> np.ndarray:
    """Returns the range image given in the `frame` captured by `lidar_name`.

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
    # Here we perform a contrast adjustment using the 1- and 99-percentile
    ri_inten_scaled = ri_intensity * np.amax(ri_intensity) / 2
    scale_factor_intensity = np.amax(ri_intensity) - np.amin(ri_intensity)
    ri_intensity = ri_inten_scaled * 255 / scale_factor_intensity
    ri_intensity = ri_intensity.astype(np.uint8)
    ### Step 6 : Stack the range and intensity image vertically using np.vstack
    # Then convert the result to unsigned 8-bit integer type
    img_range_intensity = np.vstack((ri_range, ri_intensity))
    img_range_intensity = img_range_intensity.astype(np.uint8)
    # Return resulting range image
    # intended to be visualised inside `loop_over_dataset.py` using Open3D
    ####### ID_S1_EX1 END #######
    return img_range_intensity


### Create Bird's-Eye View of LiDAR data (ID_S2_EX1)
def bev_from_pcl(
        lidar_pcl: np.ndarray, configs: easydict.EasyDict, vis: bool=True
):
    """Converts the point cloud to a BEV map.

    :param lidar_pcl: the point cloud to clip and convert to BEV map.
    :param configs: the EasyDict instance storing the viewing range and
        filtering params used to crop, discretise and re-scale the PCL values.
    :param vis: boolean (optional), If True, will visualise the resulting BEV
        map using the Open3D `Visualizer` interface.
    :returns: input_bev_maps, the 3-channel RGB-like BEV map tensor, the
        structure is as follows: `[intensity, height, density]`.
    """

    ### Step 0 : Pre-processing
    pcl_range = lidar_pcl[:, 0]
    pcl_intensity = lidar_pcl[:, 1]
    pcl_elongation = lidar_pcl[:, 2]
    # Crop LiDAR point cloud to desired region of interest (ROI)
    # And remove points with low reflectivity values
    mask = np.where(
        (pcl_range >= configs.lim_x[0]) & (pcl_range <= configs.lim_x[1]) &
        (pcl_intensity >= configs.lim_y[0]) & (pcl_intensity <= configs.lim_y[1]) &
        (pcl_elongation >= configs.lim_z[0]) & (pcl_elongation <= configs.lim_z[1])
    )
    lidar_pcl = lidar_pcl[mask]
    # Shift level of ground plane to avoid flipping from 0 to 255 for neighbouring pixels
    lidar_pcl[:, 2] = lidar_pcl[:, 2] - configs.lim_z[0]
    ####### ID_S2_EX1 START #######
    ### Summary: Convert sensor coordinates to BEV map coordinates
    print("student task ID_S2_EX1")
    ### Step 1 :  Compute BEV map discretisation
    # Here we divide the x-range by the BEV image height
    bev_interval = (configs.lim_x[1] - configs.lim_x[0]) / configs.bev_height
    bev_offset = (configs.bev_width + 1) / 2
    ### Step 2 : Transform all matrix x-coordinates into BEV coordinates
    # Here we create a copy to avoid mutation of the original PCL
    lidar_pcl_cpy = lidar_pcl.copy()
    lidar_pcl_cpy[:, 0] = np.int_(np.floor(lidar_pcl_cpy[:, 0] / bev_interval))
    ### Step 3 : Perform the same operation as in Step 2 for the y-coordinates
    # Here we make sure that no negative BEV coordinates occur
    lidar_pcl_cpy[:, 1] = np.int_(
        np.floor(lidar_pcl_cpy[:, 1] / bev_interval) + bev_offset
    )
    lidar_pcl_cpy[lidar_pcl_cpy < 0.0] = 0.0
    ### Step 4 : Visualise point cloud using `show_pcl` from task ID_S1_EX2
    if vis:
        show_pcl(lidar_pcl_cpy)
    ####### ID_S2_EX1 END #######     
    ####### ID_S2_EX2 START #######
    ### Summary: Compute intensity layer of the BEV map
    print("student task ID_S2_EX2")
    ### Step 0: Pre-processing
    # Here we clip the intensity map values to 1.0
    lidar_pcl_cpy[lidar_pcl_cpy[:, 3] > 1.0, 3] = 1.0
    ### Step 1 : Create a Numpy array filled with zeros
    # Here we specify the dimensions of the BEV map
    intensity_map = np.zeros(shape=(configs.bev_height, configs.bev_height))
    ### Step 2 : Re-arrange elements in `lidar_pcl_cpy`
    # Here we sort first by x, then y, then -z (decreasing height values)
    # Here we use `numpy.lexsort` function to accomplish this ordering
    idxs_intensity = np.lexsort(
        keys=(-lidar_pcl_cpy[:, 2], lidar_pcl_cpy[:, 1], lidar_pcl_cpy[:, 0])
    )
    lidar_pcl_top = lidar_pcl_cpy[idxs_intensity]
    ### Step 3 : Extract all points with identical x and y,
    #            s.t. only top-most z-coordinate is kept
    # Here we store the number of points per x,y-cell for use in the next task
    _, idxs_top_unique, counts = np.unique(
            lidar_pcl_top[:, 0:2], axis=0, return_index=True, return_counts=True
    )
    lidar_pcl_top = lidar_pcl_cpy[idxs_top_unique]
    ### Step 4 : Assign the intensity map values
    # Here the intensity value of each unique `lidar_top_pcl` entry is assigned
    # The intensity values are scaled s.t. objects of interest are clearly visible
    # using a min-max normalisation approach.
    intensity_vals = lidar_pcl_top[:, 3]
    scale_factor_intensity = np.amax(intensity_vals) - np.amin(intensity_vals)
    intensity_map[np.int_(lidar_pcl_top[:, 0]),
                  np.int_(lidar_pcl_top[:, 1])
                 ] = lidar_pcl_top[:, 3] / scale_factor_intensity
    ### Step 5 : Temporarily visualise the intensity map using OpenCV
    # Here we make sure that vehicles separate well from the background
    img_intensity = intensity_map * 256
    img_intensity = img_intensity.astype(np.uint8)
    str_title = "Bird's-eye view (BEV) map: normalised intensity channel values"
    cv2.imshow(str_title, img_intensity)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    ####### ID_S2_EX2 END ####### 
    ####### ID_S2_EX3 START #######
    # Summary: Compute height layer of the BEV map
    print("student task ID_S2_EX3")
    ### Step 1 : Create the BEV map array
    height_map = np.zeros((configs.bev_height, configs.bev_width))
    ### Step 2 : Assign the height map values from `lidar_top_pcl`
    # Here we use the min-max normalised values of each unique entry
    scale_factor_height = float(np.abs(configs.lim_z[1] - configs.lim_z[0]))
    height_map[np.int_(lidar_pcl_top[:, 0]),
               np.int_(lidar_pcl_top[:, 1])
              ] = lidar_pcl_top[:, 2] / scale_factor_height
    ### Step 3 : Temporarily visualize the intensity map using OpenCV
    # Here we make sure that vehicles separate well from the background
    img_height = height_map * 256
    img_height = img_height.astype(np.uint8)
    str_title = "Bird's-eye view (BEV) map: normalised height channel values"
    cv2.imshow(str_title, img_height)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    ####### ID_S2_EX3 END #######
    ### Compute density layer of the BEV map
    density_map = np.zeros((configs.bev_height + 1, configs.bev_width + 1))
    _, _, counts = np.unique(
            lidar_pcl_cpy[:, 0:2], axis=0, return_index=True, return_counts=True
    )
    normalizedCounts = np.minimum(1.0, np.log(counts + 1) / np.log(64)) 
    density_map[np.int_(lidar_pcl_top[:, 0]),
                np.int_(lidar_pcl_top[:, 1])
                ] = normalizedCounts
    ### Create a 3-channel BEV map from the individual maps
    # Here we create a 3-channel RGB-like data structure
    bev_map = np.zeros((3, configs.bev_height, configs.bev_width))
    bev_map[2, :, :] = density_map[:configs.bev_height, :configs.bev_width]
    bev_map[1, :, :] = height_map[:configs.bev_height, :configs.bev_width]
    bev_map[0, :, :] = intensity_map[:configs.bev_height, :configs.bev_width]
    ### Expand dimension of `bev_map` before converting into a tensor
    s1, s2, s3 = bev_map.shape
    bev_maps = np.zeros((1, s1, s2, s3))
    bev_maps[0] = bev_map
    ### Create the tensor from the Bird's-Eye View (BEV) map
    bev_maps = torch.from_numpy(bev_maps)
    input_bev_maps = bev_maps.to(configs.device, non_blocking=True).float()
    return input_bev_maps