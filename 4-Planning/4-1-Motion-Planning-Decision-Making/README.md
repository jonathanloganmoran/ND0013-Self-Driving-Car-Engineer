# Project 4.1: Motion Planning and Decision Making for Autonomous Vehicles
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From the Self-Driving Car Engineer Nanodegree programme offered at Udacity.

## Objectives
* Implement a [finite-state machine](https://en.wikipedia.org/wiki/Finite-state_machine) to model behavioural planning logic;
* Perform static object [collision detection](https://en.wikipedia.org/wiki/Collision_detection);
* Use the cubic spiral to represent paths and trajectories for the [motion planner](https://en.wikipedia.org/wiki/Motion_planning);
* Design and implement [cost functions](https://en.wikipedia.org/wiki/Loss_function) that penalise collisions / proximity to obstacles / deviations from lane centre;
* Perform best-trajectory selection via cost function optimisation.

## Tasks
### Behaviour Planning ([`behavior_planner_FSM.cpp`]())
* ✅ Implement the look-ahead distance function;
* ✅ Compute a goal state (pose, velocity) situated behind a stopping point;
* ✅ Compute the goal speed for a nominal state;
* ✅ Track the existing goal when in `DECEL_TO_STOP` state;
* ✅ Calculate the distance travelled w.r.t. rectlinear motion;
* ✅ Compute `DECEL_TO_STOP` state w.r.t. distance rather than speed;
* ⬜️ Move the ego-vehicle to a `STOPPED` state;
* ⬜️ Track the existing goal when in `STOPPED` state;
* ⬜️ Move ego-vehicle to a `FOLLOW_LANE` state.

### Cost functions
* ⬜️ Evaluate cost w.r.t. placement of objects (as circles);
* ⬜️ Evaluate cost w.r.t. distance to objects (as circles);
* ⬜️ Evaluate cost w.r.t. distance between last waypoint on spiral and final goal state.

### Motion Planning
* ⬜️ Plan paths based on perpendicular heading direction;
* ⬜️ Plan paths based on goal-offset location.

### Velocity Profile Generation
* ⬜️ Compute the distance travelled w.r.t. rectilinear motion for the linear velocity profile;
* ⬜️ Compute the final velocity w.r.t. rectilinear motion for the linear velocity profile;

### Fine-Tuning Hierarchial Planner
* ⬜️ Define the number of paths (goal states) to enumerate;
* ⬜️ Define the number of waypoints to enumerate for each path spiral. 


## 1. Introduction
In this project we implement several core components of the traditional [hierarchial task network](https://en.wikipedia.org/wiki/Hierarchical_task_network)-based planner: the behaviour planner and the motion planner. Both the behaviour and the motion planner modules will be able to:
1. Avoid static objects located on the sides of the road. These obstacles (e.g., cars, bicycles, trucks) are placed such that their location intersects the lane travelled by the ego-vehicle;
    * The ego-vehicle must avoid crashing into these obstacles by executing a proper "nudge" or lane-change manoeuvre;
2. Handle intersections of arbitrary type (e.g., 3-way / 4-way intersections, round-abouts); 
    * The ego-vehicle must come to a complete stop at any of these intersections;
3. Track the lane centre line.


## 2. Programming Task
### Prerequisites
In order to make use of this project, you must have the following dependencies installed:
* [Eigen 3.3.7](https://gitlab.com/libeigen/eigen/-/releases/3.3.7);
* [Carla Simulator](https://github.com/carla-simulator/carla);
* [`rpclib` 2.2.1](https://github.com/rpclib/rpclib).

While we have chosen to omit these dependencies from this repository, you may find the archived copies on the starter code repository [here](https://github.com/udacity/nd013-c5-planning-starter/tree/master/project).

## 3. Closing Remarks
###### Alternatives
* TODO.

##### Extensions of task
* TODO.

## 4. Future Work
* TODO.

## Credits
This assignment was prepared by Munir Jojo-Verge, Aaron Brown et al., 2021 (link [here](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd0013)).

References
* TODO.


Helpful resources:
* [ND0013: C5 Planning Starter | GitHub](https://github.com/udacity/nd013-c5-planning-starter).