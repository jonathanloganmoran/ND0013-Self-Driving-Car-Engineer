# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Helper functions used to for object detection.
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
import google.protobuf
import math
import numpy as np
import os
from shapely.geometry import Polygon
import sys
from typing import List, Tuple, TypeVar, Union

### Add project directory to PYTHONPATH to enable relative imports
# Alternatively, use the `pip install ..` script with setuptools
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(
    os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__)))
)
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

### Simple Waymo Open Dataset Reader library
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2
from tools.waymo_reader.simple_waymo_open_dataset_reader import label_pb2
from tools.waymo_reader.simple_waymo_open_dataset_reader import WaymoDataFileReader

### Creating some not-so-long custom variable types for typing hints
try:
    # The list of laser / camera detections type
    RepeatedCompositeContainer = TypeVar(
        google.protobuf.pyext._message.RepeatedCompositeContainer
    )
except (ModuleNotFoundError, AttributeError):
    # Out-of-date or missing `protobuf` library
    # Defaulting to generic nested list type
    RepeatedCompositeContainer = TypeVar(
        List[list]
    )
CameraCalibration = Union[dataset_pb2.CameraCalibration, RepeatedCompositeContainer]
LaserCalibration = Union[dataset_pb2.LaserCalibration, RepeatedCompositeContainer]


def compute_beam_inclinations(
        calibration: LaserCalibration,
        height: int
) -> np.ndarray:
    """Computes the inclination angle for each beam in a range image.

    :param calibration: the `LaserCalibration` protobuf message of
        the specific LiDAR sensor instance containing the beam inclinations.
    :param height: the height of the range image.
    :returns: the inclination angles as a Numpy `ndarray`.
    """

    if len(calibration.beam_inclinations) > 0:
        return np.array(calibration.beam_inclinations)
    else:
        inclination_min = calibration.beam_inclination_min
        inclination_max = calibration.beam_inclination_max
        return np.linspace(inclination_min, inclination_max, height)


def compute_range_image_polar(
        range_image: np.ndarray,
        extrinsic: np.ndarray,
        inclination: np.ndarray
) -> np.ndarray:
    """Converts the given range image to polar coordinates.

    See: http://msl.cs.uiuc.edu/planning/node102.html.

    :param range_image: the range image to convert the coordinates of stored
        as a Numpy `ndarray`.
    :param extrinsic: the extrinsic calibration matrix used to convert
        coordinates from sensor to vehicle frame, stored as a Numpy `ndarray`.
    :param inclination: the beam inclination angles as a Numpy `ndarray`.
    :returns: the polar coordinates of the range image in vehicle space.
    """

    ### Get the height and width of the range image in pixel values
    height = range_image.shape[0]
    width = range_image.shape[1]
    ### Perform an azimuth correction
    # i.e., roll the image by the sensor yaw w.r.t. the x-axis in vehicle frame
    # Here we normalise the image between [-pi, pi]
    az_correction = math.atan2(extrinsic[1, 0], extrinsic[0, 0])
    azimuth = np.linspace(np.pi, -np.pi, width) - az_correction
    azimuth_tiled = np.broadcast_to(
                        azimuth[np.newaxis, :], (height, width)
    )
    inclination_tiled = np.broadcast_to(
                        inclination[:, np.newaxis], (height, width)
    )
    return np.stack((azimuth_tiled, inclination_tiled, range_image))


def compute_range_image_cartesian(
        range_image_polar: np.ndarray,
        extrinsic: np.ndarray,
        pixel_pose: Union[np.ndarray, None],
        frame_pose: np.ndarray
) -> np.ndarray:
    """Converts the given range image from polar to Cartesian coordinates.

    See the original Waymo Open Dataset for further details,
    i.e., `waymo_open_dataset/utils/frame_utils.py` from the official
    repository at: https://github.com/waymo-research/waymo_open_dataset.

    :param range_image_polar: the range image converted to polar coordinates
        in vehicle frame using the azimuth correction step.
    :param extrinsic: the extrinsic calibration matrix containing the
        yaw, roll, pitch rotations of the LiDAR sensor.
    :param pixel_pose: the pose to set for each pixel, specified as a
        `[B, H, W, 4, 4]` tensor. If None, this is skipped.
    :param frame_pose: the reference for the vehicle frame specified as a
        `[B, 4, 4]` tensor. This must be set when `pixel_pose` is set.
    :returns: the range image converted to Cartesian coordinates.
    """

    azimuth = range_image_polar[0]
    inclination = range_image_polar[1]
    range_image_range = range_image_polar[2]

    cos_azimuth = np.cos(azimuth)
    sin_azimuth = np.sin(azimuth)
    cos_incl = np.cos(inclination)
    sin_incl = np.sin(inclination)

    ### Convert the `[B, H, W]` coordinates
    x = cos_azimuth * cos_incl * range_image_range
    y = sin_azimuth * cos_incl * range_image_range
    z = sin_incl * range_image_range
    ### Perform the transformation to vehicle coordinate frame
    # using the `einsum` compound matrix multiplication notation
    range_image_points = np.stack([x,y,z,np.ones_like(z)])
    range_image_points = np.einsum(
            'ij,jkl->ikl', extrinsic, range_image_points
    )
    return range_image_points


def get_rotation_matrix(
        roll: np.ndarray,
        pitch: np.ndarray,
        yaw: np.ndarray
) -> np.ndarray:
    """Converts the given Euler angles to a rotation matrix.

    See: http://msl.cs.uiuc.edu/planning/node102.html.

    :param roll:
    :param pitch:
    :param yaw:
    :returns:
    """

    cos_roll = np.cos(roll)
    sin_roll = np.sin(roll)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    cos_pitch = np.cos(pitch)
    sin_pitch = np.sin(pitch)
    ones = np.ones_like(yaw)
    zeros = np.zeros_like(yaw)
    ### Form the yaw rotation matrix $R_{x}(\gamma)$
    r_roll = np.stack([
        [ones,  zeros,     zeros],
        [zeros, cos_roll, -sin_roll],
        [zeros, sin_roll,  cos_roll]])
    ### Form the pitch rotation matrix $R_{y}(\beta)$
    r_pitch = np.stack([
        [ cos_pitch, zeros, sin_pitch],
        [ zeros,     ones,  zeros],
        [-sin_pitch, zeros, cos_pitch]])
    ### Compute the yaw rotation matrix $R_{z}(\alpha)$
    r_yaw = np.stack([
        [cos_yaw, -sin_yaw, zeros],
        [sin_yaw,  cos_yaw, zeros],
        [zeros,    zeros,   ones]])
    ### Compute the 3D rotation matrix as a composition of each
    pose = np.einsum(
            'ijhw,jkhw,klhw->ilhw', r_yaw, r_pitch, r_roll)
    pose = pose.transpose(2, 3, 0, 1)
    return pose


def project_to_pointcloud(
        frame: np.ndarray,
        ri: np.ndarray,
        camera_projection: np.ndarray,
        range_image_pose: np.ndarray,
        calibration: LaserCalibration
) -> Tuple[np.ndarray, np.ndarray]:
    """Creates a point cloud in vehicle frame from the given range image.

    :param frame: the `Frame` object parsed from the Waymo Open Dataset file,
        contains the pose information i.e., `transform` matrix attribute.
    :param ri: the range image to create a point cloud from given in Cartesian
        coordinates. 
    :param camera_projection: the decompressed camera projection matrix, should
        have the form:
            `{laser_name, [camera_projection_from_first_return,
              camera_projection_from_second_return]}`.
    :param range_image_pose: the range image pixel pose for 'TOP' LiDAR sensor.
    :param calibration: the `LaserCalibration` protobuf message of
        the specific LiDAR sensor instance containing the beam inclinations.
    :returns: tuple, the first three channels of the range image converted to
        Cartesian coordinates and the first channel of the original range image
        masked where values above `0` are preserved.
    """

    beam_inclinations = compute_beam_inclinations(
            calibration=calibration,
            height=ri.shape[0]
    )
    beam_inclinations = np.flip(beam_inclinations)
    extrinsic = np.array(calibration.extrinsic.transform).reshape(4, 4)
    frame_pose = np.array(frame.pose.transform).reshape(4, 4)
    ri_polar = compute_range_image_polar(
            range_image=ri[:,:, 0],
            extrinsic=extrinsic,
            inclination=beam_inclinations
    )
    #    if range_image_pose is None:
    #        pixel_pose = None
    #    else:
    #        pixel_pose = get_rotation_matrix(
    #                       range_image_pose[:,:,0],
    #                       range_image_pose[:,:,1],
    #                       range_image_pose[:,:,2]
    #       )
    #        translation = range_image_pose[:,:,3:]
    #        pixel_pose = np.block([
    #            [pixel_pose, translation[:, :, :, np.newaxis]],
    #            [np.zeros_like(translation)[:, :, np.newaxis],
    #            np.ones_like(translation[:, :, 0])[:, :, np.newaxis, np.newaxis]]])

    ri_cartesian = compute_range_image_cartesian(
            range_image_polar=ri_polar,
            extrinsic=extrinsic,
            pixel_pose=None,
            frame_pose=frame_pose
    )
    ri_cartesian = ri_cartesian.transpose(1, 2, 0)
    mask = ri[:, :, 0] > 0
    return ri_cartesian[mask, :3], ri[mask]


def display_laser_on_image(
        img: np.ndarray,
        pcl: np.ndarray,
        vehicle_to_image: np.ndarray
):
    """Converts the point cloud into homogeneous coordinates.

    :param img: the image to project the point cloud onto.
    :param pcl: the point cloud to project into the image space.
    :param vehicle_to_image: the transformation matrix converting
        vehicle coordinates to image space.
    """
    
    # Append a new axis of ones to the end of `pcl`
    pcl1 = np.concatenate(
            (pcl, np.ones_like(pcl[:, 0:1])),
            axis=1
    )
    # Transform the point cloud to the image space
    proj_pcl = np.einsum('ij,bj->bi', vehicle_to_image, pcl1)
    # Filter the LiDAR points which are behind the camera
    mask = proj_pcl[:, 2] > 0
    proj_pcl = proj_pcl[mask]
    proj_pcl_attr = pcl_attr[mask]
    # Project the point cloud onto the image plane
    proj_pcl = proj_pcl[:, :2] / proj_pcl[:, 2:3]
    # Filter the LiDAR points which are outside the image plane
    mask = np.logical_and(
            np.logical_and(proj_pcl[:, 0] > 0, proj_pcl[:, 0] < img.shape[1]),
            np.logical_and(proj_pcl[:, 1] > 0, proj_pcl[:, 1] < img.shape[1])
    )
    proj_pcl = proj_pcl[mask]
    proj_pcl_attr = proj_pcl_attr[mask]
    # Colour code the points based on distance
    coloured_intensity = 255 * cmap(proj_pcl_attr[:, 0] / 30)
    # Draw a circle for each LiDAR point
    for i in range(proj_pcl.shape[0]):
        cv2.circle(
            image=img,
            center_coordinates=(int(proj_pcl[i, 0]), int(proj_pcl[i, 1])),
            radius=1,
            color=coloured_intensity[i]
        )


def pcl_from_range_image(
        frame: dataset_pb2.Frame,
        lidar_name: int
) -> np.ndarray:
    """Extracts the LiDAR point cloud from the range image.

    :param frame: the `Frame` instance from the Waymo Open Dataset, used to
        extract the `lasers` attribute data `laser_calibrations` extrinsic
        calibration matrix.
    :param lidar_name: the id corresponding to the name of the LiDAR sensor
        whose attributes to extract, e.g., `1` will return 'TOP' LiDAR sensor.
    :returns: points_all, the stacked point cloud and LiDAR intensity channel.
    """

    ### Extract the LiDAR data and range image captured by this LiDAR sensor id
    lidar = waymo_utils.get(frame.lasers, lidar_name)
    ri_prj_pose = waymo_utils.parse_range_image_and_camera_projection(lidar)
    range_image, camera_projection, range_image_pose = ri_prj_pose
    ### Convert the range image to a point cloud
    lidar_calib = waymo_utils.get(frame.context.laser_calibrations, lidar_name)
    pcl, pcl_attr = project_to_pointcloud(
            frame=frame,
            ri=range_image,
            camera_projection=camera_projection,
            range_image_pose=range_image_pose,
            calibration=lidar_calib
    )
    ### Stack the resulting point cloud with the range image intensity channel
    points_all = np.column_stack((pcl, pcl_attr[:, 1]))
    return points_all


def project_detections_into_bev(
        bev_map: np.ndarray,
        detections: List[list],
        configs: easydict.EasyDict,
        color: List[int]=[]
):
    """Projects the detected bounding boxes into the Bird's-Eye View image.

    :param bev_map: the BEV image map to project the bounding boxes into.
    :param detections: the array of bounding box detections, each a list of
        values in the form:
            `[id, x, y, z, h, w, l, yaw]`.
    :param configs: the `EasyDict` instance containing the list of RGB values to
        use to plot the bounding boxes for each class. Each RGB is a list in the
        form: `[R, G, B]` where each element is an integer in the range [0, 255].
    :param color: the color to plot as a list of RGB color values, e.g.,
        `[255, 255, 255]`.
    """

    ### Loop over all detections
    for row in detections:
        ### Extract the vector elements of the detection
        _id, _x, _y, _z, _h, _w, _l, _yaw = row
        ### Convert from metric into pixel coordinates
        x = (_y - configs.lim_y[0])
        x /= (configs.lim_y[1] - configs.lim_y[0]) * configs.bev_width
        y = (_x - configs.lim_x[0])
        y /= (configs.lim_x[1] - configs.lim_x[0]) * configs.bev_height
        z = _z - configs.lim_z[0]
        w = _w / (configs.lim_y[1] - configs.lim_y[0]) * configs.bev_width
        l = _l / (configs.lim_x[1] - configs.lim_x[0]) * configs.bev_height
        yaw = -_yaw
        ### Draw the object bounding box in the Bird's-Eye View (BEV) image
        if not color:
            # Use the default colour for this object class
            color = configs.obj_colors[int(_id)]
        ### Get the object bounding box corners in BEV image space
        bev_corners = np.zeros((4, 2), dtype=np.float32)
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        # The front left coordinates
        bev_corners[0, 0] = x - w / 2 * cos_yaw - l / 2 * sin_yaw
        bev_corners[0, 1] = y - w / 2 * sin_yaw + l / 2 * cos_yaw
        # The rear left coordinates 
        bev_corners[1, 0] = x - w / 2 * cos_yaw + l / 2 * sin_yaw
        bev_corners[1, 1] = y - w / 2 * sin_yaw - l / 2 * cos_yaw
        # The rear right coordinates
        bev_corners[2, 0] = x + w / 2 * cos_yaw + l / 2 * sin_yaw
        bev_corners[2, 1] = y + w / 2 * sin_yaw - l / 2 * cos_yaw
        # The front right coordinates
        bev_corners[3, 0] = x + w / 2 * cos_yaw - l / 2 * sin_yaw
        bev_corners[3, 1] = y + w / 2 * sin_yaw + l / 2 * cos_yaw
        ### Create the object bounding box using OpenCV
        corners_int = bev_corners.reshape(-1, 1, 2).astype(int)
        cv2.polylines(bev_map, [corners_int], True, color, 2)
        # Render the lines so that the front class colour is visible
        corners_int = bev_corners.reshape(-1, 2)
        cv2.line(
            image=bev_map,
            start_point=(corners_int[0, 0], corners_int[0, 1]),
            end_point=(corners_int[3, 0], corners_int[3, 1]),
            color=(255, 255, 0),
            thickness=2
        )


def validate_object_labels(
        object_labels: RepeatedCompositeContainer,
        pcl: np.ndarray,
        configs: easydict.EasyDict,
        min_num_points: int
) -> np.ndarray:
    """Extracts the labels and checks them against valid / invalid constraints.

    Here an object is marked 'invalid' as indicated by a `False` value in the
    `valid_flags` array when either the number of LiDAR points inside the object
    bounding box is less than the threshold `min_num_points`, or if the object
    is outside the specified BEV range or if marked as 'difficult' to detect.
    All other 'valid' object labels will be assigned the corresponding flag
    of `True`.

    :param object_labels: the `laser_labels` protobuf container of ground truth
        LiDAR detections and class labels to consider. 
    :param pcl: the four-channel point cloud, here the fourth channel contains
        the range image intensity values to be discarded in this function.
    :param configs: the `EasyDict` instance containing the specified BEV image
        dimensions to consider, i.e., all detections outside this range are
        marked invalid (recieve a value `False` in the `valid_flags` array).
    :param min_num_points: the minimum number of LiDAR points to enforce for
        each 'valid' object, i.e., objects whose LiDAR points inside the
        bounding box are less than this number will be marked `False`.
    :returns: the list of valid / invalid detection flags as a Numpy `ndarray`
        of boolean byte values equal to the length of the number of detections.
    """

    ### Create initial list of flags where every object is set to 'valid'
    # e.g., all objects map to a flag of value `True`
    valid_flags = np.ones(len(object_labels)).astype(bool)
    ### Mark all object labels 'invalid' that do not contain a sufficient number
    #   of LiDAR points. Here we get the transformation matrix of a given label
    #   box pose in order to convert box coordinates from vehicle-to-box space.
    vehicle_to_labels = [
            np.linalg.inv(
                waymo_utils.get_box_transformation_matrix(label.box)
            ) for label in object_labels
    ]
    vehicle_to_labels = np.stack(vehicle_to_labels)
    ### Transform the point clouds from vehicle to box label space
    # First we discard the range image intensity channel values
    pcl_no_int = pcl[:, :3]
    # Construct the homogeneous coordinate system
    pcl1 = np.concatenate(
            (pcl_no_int, np.ones_like(pcl_no_int[:, 0:1])),
            axis=1
    )
    # Transform each point cloud into label space
    # Note that each `proj_pcl` has shape `[label, LiDAR point, coordinates]`
    proj_pcl = np.einsum('lij,bj->lbi', vehicle_to_labels, pcl1)
    ### Get num. objects with LiDAR points above threshold
    # For each LiDAR point / label, check if point is inside bounding box
    # Note that mask has shape `[label, LiDAR point]`
    mask = np.logical_and.reduce(
            np.logical_and(proj_pcl >= -1, proj_pcl <= 1),
            axis=2
    )
    # Count the num. points inside each bounding box, keep those above threshold
    counts = mask.sum(1)
    # Set the objects whose boxes contain num. points above threshold to `True` 
    valid_flags = counts >= min_num_points
    ### Iterate over the object labels to find 'invalid' ground truth detections
    for index, label in enumerate(object_labels):
        ### Mark objects outside the detection range as invalid
        # Form an array of box attributes to pass as an argument to validity fn.
        label_obj = [
            label.type,
            label.box.center_x, label.box.center_y, label.box.center_z,
            label.box.height, label.box.width, label.box.length,
            label.box.heading
        ]
        # Set the object's truth value based on detection area result 
        valid_flags[index] = (
                valid_flags[index]
                and is_label_inside_detection_area(
                        label=label_obj,
                        configs=configs,
                        min_overlap=0.5
                    )                     
        )
        # Set the object's truth value based on 'difficulty' and class labels
        # i.e., marked 'invalid' if 'difficult' or not 'TYPE_VEHICLE' class
        if (
            label.detection_difficulty_level > 0
            or label.type != label_pb2.Label.Type.TYPE_VEHICLE
        ):
            valid_flags[index] = False
    ### Return the array of updated boolean flags
    return valid_flags


def convert_labels_into_objects(
        object_labels: RepeatedCompositeContainer,
        configs: easydict.EasyDict
) -> List[Union[int, float]]:
    """Converts the ground truth labels in 3D object attribute lists.

    The attributes of each 'TYPE_VEHICLE' class object are parsed and stored
    into a Python list. Then, each 'TYPE_VEHICLE' object's bounding box
    coordinates are checked against the specified BEV image dimensions given
    in `configs`. If the object's bounding box coordinates are inside the
    detection area, then the corresponding attribute list is appended to the
    `detections` list to be returned.

    :param object_labels: the `laser_labels` protobuf container of ground truth
        LiDAR detections and class labels to consider.
    :param configs: the `EasyDict` instance containing the BEV image
        dimensions to enforce for the returned bounding boxes.
    :returns: the list containing the list of attributes for each object label
        of class 'TYPE_VEHICLE' with coordinates inside the specified BEV image
        dimensions.
    """
    
    detections = []
    ### Loop over all object labels
    for label in object_labels:
        ### For each label of the 'TYPE_VEHICLE' class
        if label.type == 1:
            # Create a candidate list of this object's attributes 
            candidate = [
                label.type,
                label.box.center_x, label.box.center_y, label.box.center_z,
                label.box.height, label.box.width, label.box.length,
                label.box.heading
            ]
            # Check if this object's bounding box is within the BEV dimensions    
            if(
                is_label_inside_detection_area(candidate, configs)
            ):
                # Append the candidate to the detections list if within
                # the specified BEV image dimensions
                detections.append(candidate)
    ### Return the list of attribute lists that pass the checks
    return detections


def compute_box_corners(
        x: float,
        y: float,
        w: float,
        l: float,
        yaw: float
) -> List[Tuple[float]]:
    """Computes the bounding box corners in BEV image space.

    :param x: the x-coordinate of the bounding box centre point.
    :param y: the y-coordinate of the bounding box centre point.
    :param w: the width of the bounding box from the centre point.
    param l: the length of the bounding box from the centre point.
    :param yaw: the yaw angle of the bounding box w.r.t. the vehicle frame.
    :returns: list, the list of converted bounding box corner coordinates.
    """

    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    # The front left corner coordinates
    fl = (x - w / 2 * cos_yaw - l / 2 * sin_yaw,
          y - w / 2 * sin_yaw + l / 2 * cos_yaw)
    # The rear left corner coordinates
    rl = (x - w / 2 * cos_yaw + l / 2 * sin_yaw,
          y - w / 2 * sin_yaw - l / 2 * cos_yaw)
    # The rear right corner coordinates
    rr = (x + w / 2 * cos_yaw + l / 2 * sin_yaw,
          y + w / 2 * sin_yaw - l / 2 * cos_yaw)
    # The front right corner coordinates
    fr = (x + w / 2 * cos_yaw - l / 2 * sin_yaw,
          y + w / 2 * sin_yaw + l / 2 * cos_yaw)
    return [fl, rl, rr, fr]


def is_label_inside_detection_area(
        label: List[Union[int, float]],
        configs: easydict.EasyDict,
        min_overlap: float=0.5
) -> bool:
    """Checks whether a label is inside the given BEV image detection area.

    A given label is returns `True` if its bounding box is within the BEV image
    dimensions as specified in the `configs.lim_` attributes for `x`, `y`, `z`,
    and if the bounding box has an intersecting overlap with the detection area
    of at least `min_overlap`. Otherwise, the label will be returned as `False`.

    :param label: the list of attributes of the object of interest in the form:
        `[id, x, y, z, h, w, l, yaw]`.
    :param configs: the `EasyDict` instance containing the BEV image dimensions
        to check all objects against. If any bounding box has an intersecting
        area of at least `min_threshold` inside this region, then it will be
        marked `False`.  
    :returns: bool, whether or not the object has a bounding box within the
        detection limits given by its intersection of at least `min_overlap`.
    """

    ### Convert the object label into a Polygon instance
    _, x, y, _, _, w, l, yaw = label
    # Return the bounding box corners in BEV imge space
    label_obj_corners = compute_box_corners(x, y, w, l, yaw)
    # Instantiate a Polygon instance used to check the intersection
    label_obj_poly = Polygon(label_obj_corners)   
    ### Convert the BEV image detection space into a Polygon instance
    # The width and height of the BEV detection area
    da_w = (configs.lim_x[1] - configs.lim_x[0])
    da_l = (configs.lim_y[1] - configs.lim_y[0])
    # The centre coordinate of the BEV detection area
    da_x = configs.lim_x[0] + da_w / 2
    da_y = configs.lim_y[0] + da_l / 2
    # Return the bounding box corners in BEV image space
    da_corners = compute_box_corners(da_x, da_y, da_w, da_l, 0)
    # Instantiate a Polygon instance used to check the intersection
    da_poly = Polygon(da_corners)
    ### Compute the intersection of the object and detection area
    intersection = da_poly.intersection(label_obj_poly)
    overlap = intersection.area / label_obj_poly.area
    ### Return True if the overlap is at least `min_overlap`
    return False if(overlap <= min_overlap) else True


def extract_front_camera_image(
        frame: dataset_pb2.Frame
) -> np.ndarray:
    """Extracts the RGB image from the frame captured by the 'FRONT' camera.

    :param frame: the `Frame` instance from the Waymo Open Dataset, used to
        extract the RGB images from.
    :returns: image, the RGB image captured by the 'FRONT' camera.
    """

    ### Extract the camera attributes / calibration matrix from the `frame`
    # Using only the 'FRONT' camera image and calibration data
    camera_name = dataset_pb2.CameraName.FRONT
    camera = waymo_utils.get(frame.images, camera_name)
    ### Decode and convert the RGB image
    image = waymo_utils.decode_image(camera)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    ### Return the decoded RGB image from the 'FRONT' camera
    return image


def show_bev(
        bev_maps: np.ndarray,
        configs: easydict.EasyDict
):
    """Render the BEV image map in an OpenCV window.

    :param bev_maps: the BEV map 
    :param configs: the `EasyDict` instance containing the BEV image dimensions
        to resize the given BEV image map with. 
    """

    ### Transpose the BEV image map dimensions according to the order of the
    # indices provided, e.g., permute(1,2,0) on a [1x5x2] matrix becomes [5x2x1]
    bev_map = (bev_maps.squeeze().permute(1, 2, 0).numpy() * 255).astype(np.uint8)
    bev_map = cv2.resize(bev_map, (configs.bev_width, configs.bev_height))
    bev_map = cv2.rotate(bev_map, cv2.ROTATE_180)
    ### Render the BEV image map in a new OpenCV window
    cv2.imshow('BEV map', bev_map)


def show_objects_labels_in_bev(
        detections: List[list],
        object_labels: RepeatedCompositeContainer,
        bev_maps: np.ndarray,
        configs: easydict.EasyDict
):
    """Visualises the ground-truth labels projected into the BEV image map.

    :param detections: the list of object bounding boxes to render into the
        BEV image map, each detection is a list of attributes in the form:
            `[id, x, y, z, h, w, l, yaw]`.
    :param object_labels: the `laser_labels` protobuf container of ground truth
        LiDAR detections and class labels to visualise.
    :param bev_maps: the BEV map image to project the bounding boxes into.
    :param configs: the `EasyDict` instance used to resize the BEV map to
        the specified dimensions.
    """

    ### Project the detections and ground-truth bounding boxes into BEV space
    # Note that `permute(1, 2, 0)` transforms a matrix e.g., [1x5x2] to [5x2x1]
    bev_map = (
        bev_maps.squeeze().permute(1, 2, 0).numpy() * 255
    ).astype(np.uint8)
    bev_map = cv2.resize(
        src=bev_map,
        dsize=(configs.bev_width, configs.bev_height)
    )
    label_detections = convert_labels_into_objects(
        object_labels=object_labels,
        configs=configs
    )
    project_detections_into_bev(
        bev_map=bev_map,
        detections=label_detections,
        configs=configs,
        color=[0, 255, 0]   # Color to plot the ground-truth labels with
    )
    project_detections_into_bev(
        bev_map=bev_map, 
        detections=detections, 
        configs=configs,
        color=[0, 0, 255]   # Color to plot the detections with
    )
    ### Display the resulting BEV image in a new OpenCV window
    bev_map = cv2.rotate(bev_map, cv2.ROTATE_180)
    cv2.imshow(
        'Labels (green) Versus Detected Objects (red)',
        bev_map
    )


def show_objects_in_bev_labels_in_camera(
        detections: List[list],
        bev_maps: np.ndarray,
        image: np.ndarray,
        object_labels: RepeatedCompositeContainer,
        object_labels_valid: np.ndarray,
        camera_calibration: CameraCalibration,
        configs: easydict.EasyDict
):
    """Visualises the ground truth and detections projected into BEV space.

    :param detections: the list of bounding box predictions, each detection is a
        list of attributes in the form:
            `[id, x, y, z, h, w, l, yaw]`.
    :param bev_maps: the BEV map image to project the bounding boxes into.
    :param image: the RGB image to project the bounding boxes into.
    :param object_labels: the ground-truth bounding boxes and label attributes.
    :param object_labels_valid: the list of boolean flags corresponding to each
        ground-truth object label, i.e., `False` if invalid, `True` if valid.
    :param camera_calibration: the `CameraCalibration` protobuf container storing
        the camera extrinsic calibration matrix used to transform the detections
        into the camera frame.
    :param configs: the `EasyDict` instance used to resive the BEV map to
        the specified dimensions.
    """

    ### Project the detections into Bird's-Eye View (BEV) image space
    # Note that `permute(1, 2, 0)` transforms a matrix e.g., [1x5x2] to [5x2x1]
    bev_map = (
        bev_maps.squeeze().permute(1, 2, 0).numpy() * 255
    ).astype(np.uint8)
    bev_map = cv2.resize(
        src=bev_map,
        dsize=(configs.bev_width, configs.bev_height)
    )
    project_detections_into_bev(
        bev_map=bev_map,
        detections=detections,
        configs=configs,
        color=[]    # Use the default colour for the classes
    )
    bev_map = cv2.rotate(bev_map, cv2.ROTATE_180)
    ### Project the ground-truth labels into camera image space
    img_rgb = project_labels_into_camera(
        camera_calibration=camera_calibration,
        image=image,
        labels=object_labels,
        labels_valid=object_labels_valid,
        img_resize_factor=1.0
    )
    ### Merge the camera image and BEV image into a combined view
    # Resize the RGB image
    img_rgb_h, img_rgb_w = img_rgb.shape[:2]
    ratio_rgb = configs.output_width / img_rgb_w
    output_rgb_h = int(ratio_rgb * img_rgb_h)
    ret_img_rgb = cv2.resize(
        src=img_rgb, 
        dsize=(configs.output_width, output_rgb_h)
    )
    # Resize the BEV image
    img_bev_h, img_bev_w = bev_map.shape[:2]
    ratio_bev = configs.output_width / img_bev_w
    output_bev_h = int(ratio_bev * img_bev_h)
    ret_img_bev = cv2.resize(
        src=bev_map,
        dsize=(configs.output_width, output_bev_h)
    )
    # Merge both the RGB and the BEV image into a vertically-stacked RGB image
    out_img = np.zeros(
        (output_rgb_h + output_bev_h, configs.output_width, 3),
        dtype=np.uint8
    )
    out_img[:output_rgb_h, ...] = ret_img_rgb
    out_img[output_rgb_h:, ...] = ret_img_bev
    ### Display the resulting image in a new OpenCV window
    cv2.imshow(
        'Labels Versus Detected Objects', 
        out_img
    )


def project_labels_into_camera(
        camera_calibration: CameraCalibration,
        image: np.ndarray,
        labels: List[list],
        labels_valid: np.ndarray,
        img_resize_factor: float=1.0
) -> np.ndarray:
    """Visualises the object labels projected into the RGB camera image.


    :param camera_calibration: the `CameraCalibration` protobuf container storing
        the camera extrinsic calibration matrix used to transform the detections
        into the camera frame.
    :param image: the RGB image to project the bounding boxes into.
    :param object_labels: the ground-truth bounding boxes and label attributes.
    :param object_labels_valid: the list of boolean flags corresponding to each
        ground-truth object label, i.e., `False` if invalid, `True` if valid.
    :img_resize_factor: the scale factor used to resize the RGB image.
    """

    ### Extract the vehicle-to-image frame transformation matrix
    vehicle_to_image = waymo_utils.get_image_transform(camera_calibration)
    ### Draw all 3D object labels
    for label, vis in zip(labels, labels_valid):
        if vis:
            # If the object was marked 'valid'
            colour = (0, 255, 0)
        else:
            # If the object was marked 'invalid'
            colour = (255, 0, 0)
        # Only show labels of the 'TYPE_VEHICLE' object class
        if (
            label.type == label_pb2.Label.Type.TYPE_VEHICLE
        ):
            waymo_utils.draw_3d_box(
                img=image,
                vehicle_to_image=vehicle_to_image,
                label=label,
                colour=colour
            )
        else:
            # Skip over object label if not of 'TYPE_VEHICLE' class
            continue
    ### Resize the image if specified then return the resulting image
    if (img_resize_factor < 1.0):
        width = int(image.shape[1] * img_resize_factor)
        height = int(image.shape[0] * img_resize_factor)
        dim = (width, height)
        img_resized = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
        return img_resized
    else:
        return image