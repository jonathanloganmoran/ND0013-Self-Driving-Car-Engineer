# Self-Driving Car Engineer Nanodegree
## Course 2: Sensor Fusion
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From the Self-Driving Car Engineer Nanodegree programme offered at Udacity.

This is Course 2: Sensor Fusion in the Self-Driving Car Engineer Nanodegree programme taught by Dr. Antje Muntzinger and Dr. Andreas Haja.


### Course Objectives
* Develop a strong understanding of the important role LiDAR plays in the autonomous vehicle;
* Learn to work with LiDAR range data, 3D point clouds and bird's-eye view (BEV) maps;
* Build 3D object detection models using deep learning with LiDAR point cloud data;
* Perform multi-target tracking with the Extended Kalman Filter (EKF) on multi-modal sensor data;
* Apply learnings to complete two real-world AD/ADAS detection and tracking software programmes. 


### Demo Notebooks
* ✅ [`2022-10-20-Course-2-Sensor-Fusion-Exercises-Part-1.ipynb`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/2-Sensor-Fusion/Exercises/2022-10-20-Course-2-Sensor-Fusion-Exercises-Part-1.ipynb);
* ✅ [`2022-11-11-Course-2-Sensor-Fusion-Exercises-Part-2.ipynb`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/2-Sensor-Fusion/Exercises/2022-11-11-Course-2-Sensor-Fusion-Exercises-Part-2.ipynb);
* ✅ [`2022-11-17-Course-2-Sensor-Fusion-Exercises-Part-3.ipynb`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/2-Sensor-Fusion/Exercises/2022-11-17-Course-2-Sensor-Fusion-Exercises-Part-3.ipynb);


### Projects
* ✅ 2.1: [3D Object Detection with LiDAR Data](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/2-Sensor-Fusion/2-1-3D-Object-Detection-with-LiDAR-Data);
* ⬜️ 2.2: Multi-Target Tracking with Extended Kalman filter (MTT with EKF) — in progress! 🎉


### Exercises
* ✅ [2.1.1: The LiDAR Sensor](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/2-Sensor-Fusion/Exercises/2-1-Lidar-Sensor);
* ✅ [2.1.2: Detecting Objects in LiDAR](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/2-Sensor-Fusion/Exercises/2-2-Object-Detection);
* ✅ [2.2.3: Extended Kalman Filters (EKF) for LiDAR and Camera Data](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/2-Sensor-Fusion/Exercises/2-3-Extended-Kalman-Filters/exercises);
* ✅ [2.2.4: Multi-Target Tracking (MTT) with Sensor Fusion](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/2-Sensor-Fusion/Exercises/2-4-Multi-Target-Tracking/exercises).


### Course Contents
The following topics are covered in course exercises:
* Extracting and transforming LiDAR range data
* Performing vehicle-sensor calibration
* Scaling LiDAR range intensity values using heuristic methods
* Correcting the azimuth angles using extrinsic calibration matrix
* Transforming range images to 3D point clouds
* Defining object motion model assumptions / design parameters for self-driving cars
* Using Kalman Filters for tracking in 1- and 2-D
* Linearising the camera measurement model
* Building Extended Kalman Filters (EKF) for tracking with camera and LiDAR data
* And much more ... (will be announced as course progresses)


Other topics covered in course lectures and reading material:
* Importance of multi-modal sensors in autonomous vehicles
* Scanning / Solid-State / OPA / MEMS / FMCW LiDAR specifications
* Time-of-Flight (ToF) LiDAR operating principle and equation (range, power, PBEA)
* Light propagation angle for OPA LiDAR systems
* Comparing camera / LiDAR / radar / ultrasonics performance
* Selecting the best sensor(s) for a given job with constraints
* In-depth coverage of Kalman filter update / predict equations
* Assumptions of Kalman filter equations for self-driving cars
* And much more ... (will be announced as course progresses)


### Learning Outcomes
#### Lesson 1: Introduction to Sensor Fusion and Perception
* Distinguish the strengths and weaknesses of each sensor modality;
* Understand how each sensor contributes to autonomous vehicle perception systems;
* Pick the most adequate sensor and model for a particular perception task.

#### Lesson 2: The LiDAR Sensor
* Explain the role and importance of LiDAR in autonomous driving systems;
* Extract LiDAR range data from the Waymo Open Dataset;
* Extract LiDAR attributes e.g., point correspondences, effective FOVs and calibration data;
* Visualise and scale the LiDAR range and intensity data;
* Transform the range data by e.g., cropping to ROIs, converting to 3D point clouds.

#### Lesson 3: Detecting Objects in LiDAR Data
* Transform 3D point clouds into bird's-eye view (BEV) maps;
* Perform model inference using BEV maps;
* Visualise the detection results;
* Evaluate object detection model performance with metrics;
* Experiment with state-of-the-art (SOTA) models and compare their performances.

#### Lesson 4: Kalman Filters
* Learn to track objects over time with linear Kalman filters;
* Compare tracking solutions (Kalman filters, Monte-Carlo localisation, Particle filters);
* Define measurement and motion models;
* Reason about radar measurement data to perform vehicle correspondence.

#### Lesson 5: Extended Kalman Filters
* Track objects over time with Extended Kalman Filters (EKF);
* Implement linear and non-linear motion and measurement models;
* Derive a Jacobian matrix for non-linear models;
* Apply appropriate coordinate transformations (e.g., sensor to vehicle / world coordinate space);
* Fuse LiDAR measurements with camera detections using appropriate camera models.

#### Lesson 6: Multi-Target Tracking
* Initialise, update and delete object tracks;
* Define and implement a track score and track state;
* Calculate a simple detection probability / visibility reasoning;
* Associate tracking measurements to perform Multi-Target Tracking (MTT);
* Reduce association complexity through a gating method;
* Evaluate tracking performance through metrics, e.g., RMSE.


### Material
Syllabus:
* [Program Syllabus | Udacity Nanodegree](https://d20vrrgs8k4bvw.cloudfront.net/documents/en-US/Self-Driving+Car+Engineer+Nanodegree+Syllabus+nd0013+.pdf).

Literature:
* See specific assignments for related literature.

Datasets:
* [Waymo Open Dataset: Perception](https://waymo.com/open/).

Lectures:
* Lecture materials (videos, slides) for Lessons 1 - 3 are available offline;
* Lecture matrerials (videos) for Lesson 5 are available [here](https://www.youtube.com/playlist?list=PL6nu8g-5OMNgl_rtYmrGa-K4lkjcTJbL5);
* Lecture materials (videos) for Lesson 6 are available [here](https://youtube.com/playlist?list=PL6nu8g-5OMNiT23So1PBXuL8B8HmBPpEJ).
* Course lecture notes available on request.

### Other resources
Companion code:
* [Sensor Fusion and Tracking | Starter code](https://github.com/udacity/nd013-c2-fusion-starter).
