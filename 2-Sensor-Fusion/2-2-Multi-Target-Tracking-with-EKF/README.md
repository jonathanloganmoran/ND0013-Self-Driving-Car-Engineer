# Project 2.2: Multi-Target Tracking with Extended Kalman Filter
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From Course 2: Sensor Fusion in the Self-Driving Car Engineer Nanodegree programme offered at Udacity.


## Objectives
* Write the core functions of an [extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter);
* Implement a multi-target tracking programme with track management and data association modules;
* Use a Single Nearest Neighbor (SNN) algorithm with valdiation gating to solve the data assocation problem;
* Apply mid-level sensor fusion techniques to fuse LiDAR data with detections from a non-linear camera measurement model;
* Evaluate your MTT programme on the [Waymo Open Dataset](https://waymo.com/open) and visualise the tracking results.


## Introduction
In this project you will apply the skills you have gained in this course to implement a sensor fusion system capable of tracking multiple vehicles over time. You will be provided a dataset of real-world 3D LiDAR detections extracted from the driving scenes in the [Waymo Open Dataset](https://waymo.com/open) [1]. In this project you will be fusing the LiDAR detections with the camera detections obtained from your object detection net in [Project 2.1](). You will then use these fused measurements to perform multi-target tracking with your very own non-linear [extended Kalman filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter). Together with a comprehensive track management and data association module, your programme should be able to successfully track vehicles across suburban driving environments. Let's get started!

## Prerequisites
### Extended Kalman Filter (EKF)
* ✅ [Understand the differences between Kalman filters in 1- and 2-D](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-2-MTT-with-EKF/2-Sensor-Fusion/Exercises/2022-11-11-Course-2-Sensor-Fusion-Exercises-Part-2.ipynb);
* ✅ [Formulate the measurement / state transition functions and motion models](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-2-MTT-with-EKF/2-Sensor-Fusion/Exercises/2022-11-11-Course-2-Sensor-Fusion-Exercises-Part-2.ipynb);
* ✅ [Derive the estimation error / measurement / process noise covariance matrices](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-2-MTT-with-EKF/2-Sensor-Fusion/Exercises/2022-11-11-Course-2-Sensor-Fusion-Exercises-Part-2.ipynb);
* ✅ [Implement the Extended Kalman Filter with predict / update steps](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-2-MTT-with-EKF/2-Sensor-Fusion/Exercises/2022-11-11-Course-2-Sensor-Fusion-Exercises-Part-2.ipynb);

### Multi-Target Tracking (MTT)
* ✅ [Implement the data association module with Single Nearest Neighbor (SNN) algorithm](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-2-MTT-with-EKF/2-Sensor-Fusion/Exercises/2022-11-17-Course-2-Sensor-Fusion-Exercises-Part-3.ipynb);
* ✅ [Use validation gating with Mahalanobis distance estimation error covariance](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-2-MTT-with-EKF/2-Sensor-Fusion/Exercises/2022-11-17-Course-2-Sensor-Fusion-Exercises-Part-3.ipynb);
* ✅ [Implement a track management module](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-2-MTT-with-EKF/2-Sensor-Fusion/Exercises/2022-11-17-Course-2-Sensor-Fusion-Exercises-Part-3.ipynb);

### Sensor Fusion
* ✅ [Obtain bounding box predictions with a 3D object detection net](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/2-Sensor-Fusion/2-1-3D-Object-Detection-with-LiDAR-Data/2022-11-06-Project-Writeup.md);
* ✅ [Implement a non-linear measurement camera measurement model](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-2-MTT-with-EKF/2-Sensor-Fusion/Exercises/2022-11-17-Course-2-Sensor-Fusion-Exercises-Part-3.ipynb);
* ✅ [Derive vehicle-to-sensor and sensor-to-vehicle coordinate transformation matrices](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-2-MTT-with-EKF/2-Sensor-Fusion/Exercises/2022-11-17-Course-2-Sensor-Fusion-Exercises-Part-3.ipynb);

### Improving Performance
* ⬜️ [Evaluate the tracking results with e.g., Root Mean Square Error (RMSE)]() — completed in Project 2.2!
* ⬜️ [Handle object occlusions with e.g., occlusion reasoning / probabilistic methods]() — completed in Project 2.2!

## File Descriptions

Filename                                                   | Description
-----------------------------------------------------------|----------------
`setup.py`                                                 | Builds the project, installs dependencies and places all modules onto the PYTHONPATH via `pip install -e` command (see below).
`loop_over_dataset.py`                                     | Entry point to the programme, implements all functions needed to parse, preview, modify, and evaluate on the LiDAR range images.
`data/filenames.txt`                                       | List of Google Cloud Storage (GCS) file paths to the `.tfrecord` used for model evaluation.
`student/filter.py`                                        | Implements the extended Kalman filter class and its predict / update steps.
`student/trackmanagement.py`                               | Performs track management with two classes: `Track` and `TrackManagement`; implements the track state / covariance attributes and unassigned tracks / measurements lists.
`student/association.py`                                   | Performs data association with the `Association` class; calls the EKF update function and implements the association matrix / Mahalanobis distance / gating formula.
`misc/measurements.py`                                     | Performs measurement handling with two classes: `Sensor` and `Measurement`; implements linear LiDAR / non-linear camera model, covariances, coordinate transformation matrices and measurement vectors.
`misc/params.py`                                           | Tracking parameters; defines e.g., time-step, initialisation parameters, gating threshold, etc.


## Setup and Installation
To configure your workspace, run:
```
pip install --editable {BASE_DIR}
```
where `BASE_DIR` points to the top-level project directory. All project modules will be installed onto your PYTHONPATH automatically.


## Data Acquisition

This project relies on three `.tfrecord` files from the Waymo Open Dataset for data analysis, pre-processing, inference and evaluation. These filenames are referenced in [`data/waymo_open_dataset/filenames.txt`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/2-Sensor-Fusion/2-1-3D-Object-Detection-with-LiDAR-Data/data/waymo_open_dataset/filenames.txt).

See [Project 1.1: Object Detection in Urban Environments](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/1-Computer-Vision/1-1-Object-Detection-in-Urban-Environments) for information on downloading and processing the Waymo Open Dataset files from their Google Cloud Storage (GCS) hosted bucket. You will need to request access to the dataset ahead of time [here](https://waymo.com/open/licensing).

## Tasks

### Extended Kalman Filter (EKF)
* ✅ [Implement the predict / update steps of the EKF](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/student/filter.py);
* ✅ [Implement the state transition function, process noise covariance, and residual covariance matrices](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/student/filter.py);


### Multi-Target Tracking (MTT)
* ✅ [Create a `Track` class with track state / scoring, estimation error covariances and track attributes](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/student/trackmanagement.py);
* ✅ [Create a `TrackManagement` class that implements data association, track initialisation and deletion](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/student/trackmanagement.py);
* ✅ [Improve SNN algorithm performance with validation gating](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/student/trackmanagement.py);


### Sensor Fusion
* ✅ [Create a `Sensor` class for the state estimation attributes](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/student/measurements.py);
* ✅ [Create a `Sensor` class which implements the linear LiDAR and non-linear camera measurement models](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/student/measurements.py);
* ✅ [Evaluate the single- and multi-target tracking performance and plot the RMSE]().


### Configuring the programme

#### Initialising the parameters

In order to make use of this programme, several parameters must be modified inside the [`loop_over_dataset.py`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/loop_over_dataset.py) and [`objdet_detect.py`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/student/objdet_detect.py) files.

In [`loop_over_dataset.py`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/loop_over_dataset.py), these include:
```
MODEL_NAME                          str           Name of the pre-trained model to evaluate (can be either 'darknet' or 'fpn_resnet').
SEQUENCE_ID                         int           Sequence number of the data filename to load from the Waymo Open Dataset (can be either '1', '2', or '3').
show_only_frames                    List[int]     Range of frames from the `.tfrecord` file to look at (must be integers in format: `[stop, start]`).
exec_data                           List[str]     Data manipulation functions to execute.
exec_detection                      List[str]     Object detection functions to execute.
exec_tracking                       List[str]     Multi-target tracking functions to execute (not used in this project).
exec_visualization                  List[str]     Vsualisation functions to execute.
```

In [`objdet_detect.py`](), these include:
```
REL_RESULTS                         int           Name of the project version from which to load 'results' from (can be either 'midterm' or 'final').
SEQUENCE_ID                         int           Sequence number of the data filename to load from the Waymo Open Dataset (can be either '1', '2', or '3'); must match `loop_over_dataset.py`.
```

Once the above have been initialised, run:

```python
python3 loop_over_dataset.py
```

##### Performing target tracking

In order to perform object tracking, modify the execution lists inside [`loop_over_dataset.py`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/loop_over_dataset.py) as follows: 

```
exec_data = []
exec_detection = []
exec_tracking = ['perform_tracking']
exec_visualization = ['make_traking_movie']
```

The above configuration will plot the object tracks side-by-side with the 3D bounding boxes detected in each frame of the selected Sequence file. The results are returned in a `.avi` video file saved at the file location specified in `configs.output_video_fp`. You may change this to a different output path by modifying the corresponding attribute in the [`objdet_detect.py`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/student/objdet_detect.py) file.

In order to perform single-object tracking, I suggest changing the `configs_det.lim_y` range from e.g., `[-25, 25]` to something more narrow, e.g., `[-5, 15]`, to restrict the field of view of the `'TOP'` LiDAR sensor unit. This works well with the Sequence 2 file when setting `show_only_frames = [65, 100]`.

##### Evaluating the tracking performance

In order to evaluate the RMSE performance of the tracking programme, modify the execution lists inside [`loop_over_dataset.py`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/cd84889d6711ff450d4b8306a741113b0faca50f/2-Sensor-Fusion/2-2-Multi-Target-Tracking-with-EKF/loop_over_dataset.py) as follows:

```
exec_data = []
exec_detection = []
exec_tracking = ['perform_tracking']
exec_visualization = ['show_tracks', 'make_traking_movie']
```

If you are using the provided Sequence files in the Udacity workspace, I suggest using Sequence 1, which has pre-loaded detections for all frames between `[0, 197]`. Make sure here to have `configs_det.lim_y = [-25, 25]`, i.e., the full field of view, to obtain the best multi-target tracking performance.


## Credits
This assignment was prepared by Dr. Andreas Haja and Dr. Antje Muntzinger et al., 2021 (link [here](https://github.com/udacity/nd013-c2-fusion-starter)).

References
* [1] Sun, Pei, et al. Scalability in Perception for Autonomous Driving: Waymo Open Dataset. arXiv. 2019. [doi:10.48550/ARXIV.1912.04838](https://arxiv.org/abs/1912.04838);
* [2] Konstantinova, P. et al. A Study of a Target Tracking Algorithm Using Global Nearest Neighbor Approach. CompSysTech '03: Proceedings of the 4th International Conference on Computer Systems and Technologies. Association of Computing Machinery. pp.290-295. 2003. [doi:10.1145/973620.973668](https://doi.org/10.1145/973620.973668).

Helpful resources:
* [`2022-11-11-Course-2-Sensor-Fusion-Exercises-Part-2.ipynb` by J. L. Moran | GitHub](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/81b3a9eed386bd71d53f83931f0d5ff6c8ee497d/2-Sensor-Fusion/Exercises/2022-11-11-Course-2-Sensor-Fusion-Exercises-Part-2.ipynb);
* [`2022-11-17-Course-2-Sensor-Fusion-Exercises-Part-3.ipynb` by J. L. Moran | GitHub](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/81b3a9eed386bd71d53f83931f0d5ff6c8ee497d/2-Sensor-Fusion/Exercises/2022-11-17-Course-2-Sensor-Fusion-Exercises-Part-3.ipynb);
* [`07-Kalman-Filter-Math.ipynb` by R. Labbe | GitHub](https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb);
* [Introduction to Data Association by B. Collins | Lecture Slides](https://www.cse.psu.edu/~rtc12/CSE598C/datassocPart1.pdf).