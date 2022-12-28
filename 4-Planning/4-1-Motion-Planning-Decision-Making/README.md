# Project 4.1: Motion Planning and Decision Making for Autonomous Vehicles
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From the Self-Driving Car Engineer Nanodegree programme offered at Udacity.

## Objectives
* Implement a [finite-state machine](https://en.wikipedia.org/wiki/Finite-state_machine) to model behavioural planning logic;
* Perform static object [collision detection](https://en.wikipedia.org/wiki/Collision_detection);
* Use the cubic spiral to represent paths and trajectories for the [motion planner](https://en.wikipedia.org/wiki/Motion_planning);
* Design and implement [cost functions](https://en.wikipedia.org/wiki/Loss_function) that penalise collisions / proximity to obstacles / deviations from lane centre;
* Perform best-trajectory selection via cost function optimisation.

## 1. Introduction
In this project we implement several core components of the traditional [hierarchial task network](https://en.wikipedia.org/wiki/Hierarchical_task_network)-based planner: the behaviour planner and the motion planner. Both the behaviour and the motion planner modules will be able to:
1. Avoid static objects located on the sides of the road. These obstacles (e.g., cars, bicycles, trucks) are placed such that their location intersects the lane travelled by the ego-vehicle;
    * The ego-vehicle must avoid crashing into these obstacles by executing a proper "nudge" or lane-change manoeuvre;
2. Handle intersections of arbitrary type (e.g., 3-way / 4-way intersections, round-abouts); 
    * The ego-vehicle must come to a complete stop at any of these intersections;
3. Track the lane centre line.


## 2. Programming Task
TODO.

## 3. Closing Remarks
###### Alternatives
* TODO.

##### Extensions of task
* TODO.

## 4. Future Work
* TODO.

## Credits
This assignment was prepared by Munir Jojo-Verge, 2020 (link [here](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd0013)).

References
* TODO.


Helpful resources:
* TODO.