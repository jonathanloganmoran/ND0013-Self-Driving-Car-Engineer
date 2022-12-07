# Self-Driving Car Engineer Nanodegree
## Course 3: Localization
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From the Self-Driving Car Engineer Nanodegree programme offered at Udacity.

This is Course 3: Localization in the Self-Driving Car Engineer Nanodegree programme taught by Aaron Brown, Tiffany Huang and Maximilian Muffert of Mercedes-Benz Research & Development North America (MBRDNA).


### Course Objectives
* Master robot localisation from one-dimensional motion models to three-dimensional point cloud maps;
* Study the [bicycle motion model](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html) to improve on the [previous lesson's]() linear motion model assumption;
* Master the fundamentals of the [Bayes' theorem](https://en.wikipedia.org/wiki/Bayes%27_theorem) applied to robot localisation;
* Implement [Markov localisation](https://en.wikipedia.org/wiki/Markov_property) to perform 1D object tracking in C++;
* Implement two scan matching algorithms: [Iterative Closest Point](https://en.wikipedia.org/wiki/Iterative_closest_point) (ICP) and [Normal Distributions Transform](https://en.wikipedia.org/wiki/Normal_distributions_transform) (NDT) in C++;
* Use the [Point Cloud Library](https://en.wikipedia.org/wiki/Point_Cloud_Library) (PCL) in C++ to build these scan matching algorithms and apply them to simulated LiDAR point cloud data;
* Localise a simulated vehicle with LiDAR sesning using the 3D point cloud map obtained from the [CARLA Simulator](https://carla.org/).


### Demo Notebooks
* ✅ [`2022-11-25-Course-3-Localization-Exercises-Part-1.ipynb`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/3-1-Markov-Localization/2022-11-25-Course-3-Localization-Exercises-Part-1.ipynb);
* ✅ [`2022-11-25-Course-3-Localization-Exercises-Part-2.ipynb`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/3-Localization/3-1-Markov-Localization/2022-11-25-Course-3-Localization-Exercises-Part-2.ipynb).


### Projects
* ⬜️ 3.1: [Scan Matching Localization]().


### Exercises
* ✅ [3.1: Markov Localization](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/3-Localization/3-1-Markov-Localization/exercises);
* ✅ [3.2: Point Cloud Library (PCL)](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/3-Localization/3-2-Point-Cloud-Library);
* ✅ [3.3: Scan Matching Algorithms]();
* More info coming soon...


### Course Contents
The following topics are covered in course exercises:
* More info coming soon...


Other topics covered in course lectures and reading material:
* More info coming soon...


### Learning Outcomes
#### Lesson 1: Introduction to Localization
* Understand how self-driving vehicles leverage GPS or object detections to localise itself in an environment;
* Predict vehicle motion using the [Kinematic Bicycle Motion](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/BicycleModel.html) model.

#### Lesson 2: Markov Localization
* Apply the law of total probability to robotic motion;
* Derive the general Bayes' Filter with Markov assumption;
* Implement a 1D localisation programme in C++.

#### Lesson 2.5: Introduction to Point Cloud Library
* Familiarise yourself with the [PCL](https://pointclouds.org/) library;
* Learn about the basics of PCL, e.g., the PCD file type and the PCL Viewer;
* Create and use LiDAR objects in PCL;
* Learn the relative PCL object [templates](http://www.cplusplus.com/doc/oldtutorial/templates/) and pointers in C++;
* Modify LiDAR object properties and parameters, e.g., number of layers, add random noise.

#### Lesson 3: Creating Scan Matching Algorithms
* Learn how the [Iterative Closest Point](https://en.wikipedia.org/wiki/Iterative_closest_point) (ICP) algorithm is used for localisation;
* Learn how the [Normal Distributions Transform](https://en.wikipedia.org/wiki/Normal_distributions_transform) (NDT) algorithm is used for localisation;
* Implement the ICP and NDT for 2D localisation in C++.

#### Lesson 4: Utilizing Scan Matching in 3D
* Align 3D point cloud maps using the ICP algorithm;
* Align 3D point cloud maps using the NDT algorithm;
* Create and use point cloud maps in the CARLA simulator.


### Material
Syllabus:
* [Program Syllabus | Udacity Nanodegree](https://d20vrrgs8k4bvw.cloudfront.net/documents/en-US/Self-Driving+Car+Engineer+Nanodegree+Syllabus+nd0013+.pdf).

Literature:
* See specific assignments for related literature.

Datasets:
* [Waymo Open Dataset: Perception](https://waymo.com/open/).

Lectures:
* TODO.

### Other resources
Companion code:
* TODO.
