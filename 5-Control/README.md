# Self-Driving Car Engineer Nanodegree
## Course 5: Control
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From the Self-Driving Car Engineer Nanodegree programme offered at Udacity.

This is Course 5: Control in the Self-Driving Car Engineer Nanodegree programme taught by David Siller, Andrew Gray and Sebastian Thrun.


### Course Objectives
* Design throttle- and steering-command controllers to perform vehicle trajectory tracking;
* Implement Proportional, Proportional-Derivative (PD), and Proportional-Integral-Derivative (PID) controllers in Python and C++;
* Test the PID controller efficacy in the CARLA Simulator.


### Demo Notebooks
* ✅ [`2023-01-03-Course-5-Control-Exercises-Part-1.ipynb`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/5-Control/Exercises/2023-01-03-Course-5-Control-Exercises-Part-1.ipynb).


### Projects
* ✅ [Project 5.1: Control and Trajectory Tracking for Autonomous Vehicles](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/5-Control/5-1-Control-Trajectory-Tracking).


### Exercises
* ✅ [5.1: PID Control](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/5-Control/Exercises/5-1-PID-Control).
* ✅ [5.2: Vehicle Motion Models](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/5-Control/Exercises/5-2-Vehicle-Models).


### Course Contents
The following topics are covered in course exercises:
* Vehicle state (position / velocity) and control variables (steering, acceleration, braking);
* Feedback controllers (PID, MPC) for trajectory tracking;
* Stability in the non-linear case;
* Controller robustness to real-world perturbations;
* Global kinematic motion model;
* Polynomial curve fitting;
* Model Predictive Control (MPC) for self-driving cars;
* Automatic differentiation in C++;
* Cost functions;
* Cross-track error (CTE) and heading angle error (e-psi);
* More info coming soon...

Other topics covered in course lectures and reading material:
* Basics of feedback controllers;
* Various controller principles (P-, PI- PID-controllers);
* Vehicle and actuator dynamics (e.g., tire forces models, actuator constraints / latency);
* MPC parameter tuning and trade-offs;
* More info coming soon...


### Learning Outcomes
#### Lesson 1: PID Control
* Define the vehicle observation (position, velocity) and action (steering, acceleration, braking) states of the vehicle;
* Design and code the PID controller in Python and C++;
* Design and code the MPC in C++;
* Evaluate the efficicacy of several controller types for autonomous vehicles using CARLA Simulator.

#### Lesson 2: Vehicle Motion Models
* Robot motion models (kinematic and dynamic);
* Vehicle motion models (kinematic bicycle, complex / dynamics models);
* Tire forces (slip angle / ratio);
* Pacejka tire model / "magic tire formula"; 
* Kinematic model parameters;
* Actuator constraints;
* Implementation of kinematic models and polynomial curve fitting in C++.


### Material
Syllabus:
* [Program Syllabus | Udacity Nanodegree](https://d20vrrgs8k4bvw.cloudfront.net/documents/en-US/Self-Driving+Car+Engineer+Nanodegree+Syllabus+nd0013+.pdf).

Literature:
* See specific assignments for related literature.

Datasets:
* N/A.

Lectures:
* Lectures available offline on request.

### Other resources
Companion code:
* [`nd013-c6-control-starter` by @udacity | GitHub](https://github.com/udacity/nd013-c6-control-starter/tree/master);
* [`CarND-MPC-Quizzes` by @udacity | GitHub](https://github.com/udacity/CarND-MPC-Quizzes).