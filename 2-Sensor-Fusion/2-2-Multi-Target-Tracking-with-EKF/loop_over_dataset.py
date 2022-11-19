# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Loop over all frames in a Waymo Open Dataset file,
#                        detect and track objects and visualise the results.              
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
import copy
import cv2
import math
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

### Add current working directory to path
sys.path.append(os.getcwd())

### Simple Waymo Open Dataset Reader library
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import dataset_pb2
from tools.waymo_reader.simple_waymo_open_dataset_reader import label_pb2
from tools.waymo_reader.simple_waymo_open_dataset_reader import WaymoDataFileReader

### 3D object detection imports
from misc.helpers import load_object_from_file
from misc.helpers import make_exec_list
from misc.helpers import save_object_to_file
import student.objdet_detect as det
import student.objdet_eval as evals
import student.objdet_pcl as pcl
import misc.objdet_tools as tools

### Tracking package imports
from student.filter import Filter
from student.trackmanagement import Trackmanagement
from student.association import Association
from student.measurements import Sensor, Measurement
from misc.evaluation import plot_tracks, plot_rmse, make_movie
import misc.params as params 

# Name of the model to load
MODEL_NAME = 'fpn_resnet'

### Set parameters and perform initializations
# Select Waymo Open Dataset file and frame numbers
# Sequence 1
# data_filename = 'training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord'
# Sequence 2
data_filename = 'training_segment-10072231702153043603_5725_000_5745_000_with_camera_labels.tfrecord'
# Sequence 3
# data_filename = 'training_segment-10963653239323173269_1924_000_1944_000_with_camera_labels.tfrecord'
# Restrict the number of frames to interval
show_only_frames = [150, 200]
### Prepare Waymo Open Dataset file for loading
# Paths to the 'dataset' and 'results' folders
# Using paths in case the script is called from another working directory
data_fullpath = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), 'dataset', data_filename
)
results_fullpath = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), 'results'
)
datafile = WaymoDataFileReader(data_fullpath)
datafile_iter = iter(datafile)  # initialize dataset iterator
### Initialise the object detection model
# The model can be any one of the following:
#     ['darknet', 'fpn_resnet']
configs_det = det.load_configs(model_name=MODEL_NAME)
model_det = det.create_model(configs_det)
# Update the results filepath to the relative subfolder used by the model
# Should point to e.g., '../results/fpn-resnet/results_sequence_1_resnet/'
results_fullpath = os.path.join(results_fullpath, 
    configs_det.saved_fn, configs_det.rel_results_folder
)
# If True, ground-truth labels are used as objects
# If False, run inference over objects in the detection / testing loop
configs_det.use_labels_as_objects = False
### Uncomment this setting to restrict the y-range in the final project
configs_det.lim_y = [-25, 25] 
### Initialise the MTT algorithm (Project 2.2)
# Instantiate the Kalman filter class
KF = Filter()
# Instantiate the data association class
association = Association()
# Instantiate the track manager class
manager = Trackmanagement()
# Init the LiDAR sensor object
lidar = None
# Init the camera sensor object
camera = None
# Fix the seed such that random values are predictable
np.random.seed(10)
### Selective execution and visualisation
# Set the data manipulation executions to perform, can be any of the following:
#     ['pcl_from_rangeimage', 'load_image']
exec_data = ['pcl_from_rangeimage']  
# Set the detection executions to perform, can be any of the following:
#     ['bev_from_pcl', 'detect_objects',
#      'validate_object_labels',
#      'measure_detection_performance'
#     ]
# Any options not in the list will be loaded from file.
exec_detection = []
# Set the tracking executions to perform, can be any of the following:
# ['perform_tracking']
exec_tracking = ['perform_tracking']
# Set the visualisation executions to perform;
# Can be any number of the following:
#     ['show_range_image', 'show_bev', 'show_pcl',
#      'show_labels_in_image', 'show_objects_and_labels_in_bev',
#      'show_objects_in_bev_labels_in_camera', 'show_tracks',
#      'show_detection_performance', 'make_tracking_movie'
#     ]
exec_visualization = ['show_tracks']
# Initialise the execution list for each component
# If a `Segmentation fault` error occurs, change these to positional arguments
exec_list = make_exec_list(
        exec_detection=exec_detection,
        exec_tracking=exec_tracking,
        exec_visualization=exec_visualization,
        exec_data=exec_data
)
# Set the pause time between frames (ms);
# If set to '0', stop between frames until key is pressed.
vis_pause_time = 0


####### START 3D Object Detection / Tracking #######
### Perform detection & tracking over all selected frames
cnt_frame = 0 
all_labels = []
det_performance_all = []
# Fix the seed such that random values are predictable
np.random.seed(0)
if 'show_tracks' in exec_list:
    # Initialise the figure to plot tracking results
    fig, (ax2, ax) = plt.subplots(1,2)
### Entry point into the main programme loop
while True:
    try:
        # Get next frame from Waymo dataset
        frame = next(datafile_iter)
        if cnt_frame < show_only_frames[0]:
            cnt_frame = cnt_frame + 1
            continue
        elif cnt_frame > show_only_frames[1]:
            print('Reached end of selected frames')
            break
        print('------------------------------')
        print(f'Processing frame #{cnt_frame}')

        ####### 3D Object Detection task #######
        
        # Extract calibration data from frame
        lidar_name = dataset_pb2.LaserName.TOP
        camera_name = dataset_pb2.CameraName.FRONT
        lidar_calibration = waymo_utils.get(
            frame.context.laser_calibrations, lidar_name
        )        
        camera_calibration = waymo_utils.get(
            frame.context.camera_calibrations, camera_name
        )
        # Extract front camera image from frame
        if 'load_image' in exec_list:
            image = tools.extract_front_camera_image(frame) 
        # Compute LiDAR point cloud from range image    
        if 'pcl_from_rangeimage' in exec_list:
            print('Computing point cloud from LiDAR range image')
            lidar_pcl = tools.pcl_from_range_image(frame, lidar_name)
        else:
            print('Loading LiDAR point cloud from result file')
            lidar_pcl = load_object_from_file(
                file_path=results_fullpath,
                base_filename=data_filename,
                object_name='lidar_pcl',
                frame_id=cnt_frame
            )
        # Compute LiDAR Bird's-Eye View (BEV) map
        if 'bev_from_pcl' in exec_list:
            print("Computing Bird's-Eye View (BEV) map from LiDAR point cloud")
            lidar_bev = pcl.bev_from_pcl(lidar_pcl, configs_det)
        else:
            print("Loading Bird's-Eye View (BEV) map from 'results' folder")
            lidar_bev = load_object_from_file(
                file_path=results_fullpath,
                base_filename=data_filename,
                object_name='lidar_bev',
                frame_id=cnt_frame
            )
        # Perform inference (3D object detection) over BEV map
        if (configs_det.use_labels_as_objects==True):
            print('Using ground-truth labels as objects')
            detections = tools.convert_labels_into_objects(
                frame.laser_labels, configs_det
            )
        else:
            # Perform 3D object detection over ground-truth data
            if 'detect_objects' in exec_list:
                print('Detecting objects in BEV map from LiDAR point cloud data')   
                detections = det.detect_objects(
                    lidar_bev, model_det, configs_det
                )
            else:
                # Load the already-computed evaluation results (detections)
                print("Loading detected objects from 'results' folder")
                # Load different data for final project vs. mid-term project
                if 'perform_tracking' in exec_list:
                    # Load the results needed to perform tracking task 
                    detections = load_object_from_file(
                        file_path=results_fullpath,
                        base_filename=data_filename,
                        object_name='detections',
                        frame_id=cnt_frame
                    )
                else:
                    # Load the results needed to perform detection evaluation
                    dets_fp_ext = f"detections_{configs_det.arch}_{configs_det.conf_thresh}"
                    detections = load_object_from_file(
                        file_path=results_fullpath,
                        base_filename=data_filename,
                        object_name=dets_fp_ext,
                        frame_id=cnt_frame
                    )
        # Validate object labels against ground-truth
        if 'validate_object_labels' in exec_list:
            print("Validating object labels")
            valid_label_flags = tools.validate_object_labels(
                    frame.laser_labels, 
                    lidar_pcl, 
                    configs_det,
                    0 if configs_det.use_labels_as_objects else 10
            )
        else:
            print("Loading object labels and val. data from 'results' folder")
            valid_label_flags = load_object_from_file(
                    file_path=results_fullpath, 
                    base_filename=data_filename, 
                    object_name='valid_labels', 
                    frame_id=cnt_frame
            )            
        # Performance evaluation for object detection
        if 'measure_detection_performance' in exec_list:
            print('Measuring detection performance')
            det_performance = evals.measure_detection_performance(
                    detections, 
                    frame.laser_labels, 
                    valid_label_flags, 
                    configs_det.min_iou
            )     
        else:
            print("Loading detection performance measures from 'results' folder")
            # Load different data for final project vs. mid-term project
            if 'perform_tracking' in exec_list:
                # Load the results needed for the tracking task
                det_performance = load_object_from_file(
                    file_path=results_fullpath,
                    base_filename=data_filename,
                    object_name='det_performance',
                    frame_id=cnt_frame
                )
            else:
                # Load the results needed for the tracking task
                dets_fp_ext = f"det_performance_{configs_det.arch}_{configs_det.conf_thresh}"
                det_performance = load_object_from_file(
                    file_path=results_fullpath, 
                    base_filename=data_filename, 
                    object_name=dets_fp_ext,
                    frame_id=cnt_frame
                )
        # Store all evaluation results in a list for assessment in next task
        det_performance_all.append(det_performance)
        # Visualisation for the 3D object detection task
        if 'show_range_image' in exec_list:
            # Visualise the range images
            img_range = pcl.show_range_image(frame, lidar_name)
            img_range = img_range.astype(np.uint8)
            cv2.imshow('range_image', img_range)
            cv2.waitKey(vis_pause_time)
        if 'show_pcl' in exec_list:
            # Visualise the 3D point cloud data
            pcl.show_pcl(lidar_pcl)
        if 'show_bev' in exec_list:
            # Visualise the Bird's-Eye View (BEV) map images
            tools.show_bev(lidar_bev, configs_det)  
            cv2.waitKey(vis_pause_time)          
        if 'show_labels_in_image' in exec_list:
            # Visualise the projected bounding box detections into the BEV map 
            img_labels = tools.project_labels_into_camera(
                camera_calibration,
                image,
                frame.laser_labels,
                valid_label_flags,
                0.5
            )
            cv2.imshow('img_labels', img_labels)
            cv2.waitKey(vis_pause_time)
        if 'show_objects_and_labels_in_bev' in exec_list:
            # Visualise ground-truth and detected bounding boxes in the BEV map
            tools.show_objects_labels_in_bev(
                detections,
                frame.laser_labels,
                lidar_bev,
                configs_det
            )
            cv2.waitKey(vis_pause_time)         
        if 'show_objects_in_bev_labels_in_camera' in exec_list:
            tools.show_objects_in_bev_labels_in_camera(
                detections,
                lidar_bev,
                image,
                frame.laser_labels,
                valid_label_flags,
                camera_calibration,
                configs_det
            )
            cv2.waitKey(vis_pause_time)               

        ####### Multi-Target Tracking (MTT) task #######

        # Perform the tracking task
        if 'perform_tracking' in exec_list:
            # Initialise sensor objects
            if lidar is None:
                # Init LiDAR sensor object
                lidar = Sensor('lidar', lidar_calibration)
            if camera is None:
                # Init camera sensor object
                camera = Sensor('camera', camera_calibration)
            # Pre-process the LiDAR detections (analyse for measurements)
            meas_list_lidar = []
            for detection in detections:
                # Check if measurement lies inside specified range
                if (
                    detection[1] > configs_det.lim_x[0] and
                    detection[1] < configs_det.lim_x[1] and
                    detection[2] > configs_det.lim_y[0] and 
                    detection[2] < configs_det.lim_y[1]
                ):
                    meas_list_lidar = lidar.generate_measurement(
                        cnt_frame, detection[1:], meas_list_lidar
                    )
            # Pre-process camera detections (analyse measurements / add noise)
            meas_list_cam = []
            for label in frame.camera_labels[0].labels:
                if (
                    label.type == label_pb2.Label.Type.TYPE_VEHICLE
                ):
                    box = label.box
                    # Use camera labels as measurements and add random noise
                    z = [box.center_x, box.center_y, box.width, box.length]
                    z[0] = z[0] + np.random.normal(0, params.sigma_cam_i) 
                    z[1] = z[1] + np.random.normal(0, params.sigma_cam_j)
                    meas_list_cam = camera.generate_measurement(
                        cnt_frame, z, meas_list_cam
                    )
            # Kalman prediction / filtering
            for track in manager.track_list:
                print(f'Predicting track: {track.id}')
                KF.predict(track)
                # Save the next time-stamp
                track.set_t((cnt_frame - 1) * 0.1)
            # Associate all LiDAR measurements to all tracks
            association.associate_and_update(manager, meas_list_lidar, KF)
            # Associate all camera measurements to all tracks
            association.associate_and_update(manager, meas_list_cam, KF)
            # Save results for evaluation
            result_dict = {}
            for track in manager.track_list:
                result_dict[track.id] = track
            manager.result_list.append(copy.deepcopy(result_dict))
            label_list = [frame.laser_labels, valid_label_flags]
            all_labels.append(label_list)
            # Visualise the tracking results
            if 'show_tracks' in exec_list:
                fig, ax, ax2 = plot_tracks(
                    fig, 
                    ax, 
                    ax2, 
                    manager.track_list,
                    meas_list_lidar,
                    frame.laser_labels, 
                    valid_label_flags, 
                    image, 
                    camera, 
                    configs_det
                )
                if 'make_tracking_movie' in exec_list:
                    # Save tracking results to file
                    fname = f"{results_fullpath}'/tracking{cnt_frame:03d}.png"
                    print(f"Saving frame '{fname}'")
                    fig.savefig(fname)
        ### Completing one loop
        # Increment the frame counter
        cnt_frame = cnt_frame + 1
    except StopIteration:
        ### Exit point in the programme
        # If `StopIteration` is raised, break from loop
        print("StopIteration has been raised\n")
        break

####### Post-processing tasks #######
if 'show_detection_performance' in exec_list:
    # Evaluate 3D object detection performance
    evals.compute_performance_stats(det_performance_all)
if 'show_tracks' in exec_list:
    # Evaluate tracking results by plotting RMSE for all tracks
    plot_rmse(manager, all_labels, configs_det)
if 'make_tracking_movie' in exec_list:
    # Output the tracking results to a movie file
    make_movie(results_fullpath)
