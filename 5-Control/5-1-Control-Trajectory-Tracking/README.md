# Project 5.1: Control and Trajectory Tracking for Autonomous Vehicles
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From the Self-Driving Car Engineer Nanodegree programme offered at Udacity.

## Objectives
* Design and integrate a [PID controller](https://en.wikipedia.org/wiki/PID_controller) in C++;
* Apply the PID controller to the vehicle trajectory tracking task and use it to execute steering and throttle commands;
* Test the PID controller in the vehicle using the [CARLA Simulator](http://carla.org);
* Evaluate the controller performance and efficiency and plot the results.

## Tasks
### Build the PID Controller ([`pid_controller.cpp`]())
* ⬜️ TODO.
### Use PID Controller for Vehicle Throttle ([`main.cpp`]())
* ⬜️ TODO.
### Use PID Controller for Vehicle Steering ([`main.cpp`]())
* ⬜️ TODO.
### Evaluate the PID Controller Efficiency ([`plot_pid.py`]())
* ⬜️ TODO.

## 1. Introduction
In this project we use the skills we have gained in [Course 5: Control](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/5-Control) to implement the [PID controller](https://en.wikipedia.org/wiki/PID_controller) and apply it to the vehicle trajectory tracking task. Given a trajectory and an array of locations, we will design and code a PID controller to actuate the vehicle (i.e., apply steer and throttle commands). Using the [CARLA Simulator](http://carla.org) environment, we will test the PID Controller on the ego-vehicle and evaluate its real-world performance using a simulation environment with perturbations.

The starter code for this project provides the dynamic model of the vehicle as well as an approximate distribution of the possible perturbations. All code for the CARLA simulation environment is provided in addition to a template for the controller in C++. In summary, we will complete the following steps:
1. Design the PID controller in C++;
2. Integrate the controller with the vehicle using CARLA Simulator;
3. Tune the PID controller parameters;
4. Design test- and evaluation strategies;
5. Create controller efficiency plots and test videos; 
6. Discuss the results of the PID controller, covering situations where the trajectory was recovered.

With those tasks clearly defined, let's now dive into the programming tasks for this project.

## 2. Programming Task
### 2.1. Control and Trajectory Tracking
#### Background
In this proejct we use the C++ and Python APIs for the [CARLA Simulator](http://carla.org), as well as an open-source C++ solver, to design and test a PID controller for use in vehicle trajectory tracking. The starter code for this assignment is available at the Udacity GitHub repository [here](https://github.com/udacity/nd013-c6-control-starter/tree/master). 

#### Results

TODO.

#### Prerequisites
In order to make use of this project, you must have the following dependencies installed —

Python:
* Python 3.7;
* [Carla Simulator](https://github.com/carla-simulator/carla)
* [PyPI Pip](https://pip.pypa.io/en/stable/installation/);
* [Numpy](https://numpy.org/);
* [Pygame](https://www.pygame.org/);
* [Gtest](https://pypi.org/project/gtest/).

C++:
* [C++14](https://en.wikipedia.org/wiki/C%2B%2B14)
* [Eigen 3.3.7](https://gitlab.com/libeigen/eigen/-/releases/3.3.7);
* [Carla Simulator](https://github.com/carla-simulator/carla);
* [`rpclib` 2.2.1](https://github.com/rpclib/rpclib).


#### Running and compiling the programme

##### Setting the hyperparameters
TODO.

##### Creating the executable
TODO.

##### Configuring the runtime environment
TODO.

##### Executing the programme
TODO.

##### More information
TODO.


## 3. Closing Remarks
###### Alternatives
* TODO.

##### Extensions of task
* TODO.

## 4. Future Work
* ⬜️ TODO.

## Credits
This assignment was prepared by Sebastian Thrun, David Siller, Andrew Gray et al., 2021 (link [here](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd0013)).

References
* [] TODO.

Helpful resources:
* [`nd013-c6-control-starter` by @udacity | GitHub](https://github.com/udacity/nd013-c6-control-starter/tree/master).