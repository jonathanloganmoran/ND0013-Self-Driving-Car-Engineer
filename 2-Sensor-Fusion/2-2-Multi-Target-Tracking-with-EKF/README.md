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
* ⬜️ Evaluate the tracking results with e.g., Root Mean Square Error (RMSE);
* ⬜️ Handle object occlusions with e.g., occlusion reasoning / probabilistic methods.


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

TODO.

### Multi-Target Tracking (MTT)

TODO.


### Sensor Fusion

TODO.


### Configuring the programme

TODO.

#### Initialising the parameters

In order to make use of this programme, several parameters must be modified inside the `loop_over_dataset.py` file. These include:
```
MODEL_NAME:                             str         The desired pre-trained model to evaluate (can be either 'darknet' or 'fpn_resnet').
data_filename:                          str         Name of the downloaded Waymo Open Dataset `.tfrecord` file to use (see `filenames.txt`).
show_only_frames:                       List[int]   Range of frames from the `.tfrecord` file to look at (must be integers in format: `[stop, start]`).
exec_data:                              List[str]   List of data manipulation functions to execute.
exec_detection:                         List[str]   List of object detection functions to execute.
exec_tracking:                          List[str]   List of multi-target tracking functions to execute (not used in this project).
exec_visualization:                     List[str]   List of visualisation functions to execute.
```

Once the above have been initialised, run:

```python
python3 loop_over_dataset.py
```

TODO.


#### Performing Inference and Evaluation

TODO.

## Summary
### Extended Kalman Filter (EKF)
* ⬜️ TODO .. ;

### Multi-Target Tracking (MTT)
* ⬜️ TODO .. ;

### Sensor Fusion
* ⬜️ TODO .. ;

### Improving Performance
* ⬜️ TODO .. ;


## Credits
This assignment was prepared by Dr. Andreas Haja and Dr. Antje Muntzinger et al., 2021 (link [here](https://github.com/udacity/nd013-c2-fusion-starter)).

References
* [1] Sun, Pei, et al. Scalability in Perception for Autonomous Driving: Waymo Open Dataset. arXiv. 2019. [doi:10.48550/ARXIV.1912.04838](https://arxiv.org/abs/1912.04838).

Helpful explanations:
* []();