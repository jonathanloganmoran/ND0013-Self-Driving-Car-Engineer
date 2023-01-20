# Self-Driving Car Engineer Nanodegree
## Course 4: Planning
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From the Self-Driving Car Engineer Nanodegree programme offered at Udacity.

This is Course 4: Planning in the Self-Driving Car Engineer Nanodegree programme taught by Benjamin Ulmer and Tobias Roth of Mercedes-Benz Research & Development.


### Course Objectives
* Design weighted cost functions and implement a behaviour planner in C++;
* Use a [finite-state machine](https://en.wikipedia.org/wiki/Finite-state_machine) to model vehicle / environment states and transitions;
* Implement a behaviour and motion planner in Python and C++.


### Demo Notebooks
* ✅ [`2022-12-14-Course-4-Planning-Exercises-Part-1.ipynb`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/Exercises/2022-12-14-Course-4-Planning-Exercises-Part-1.ipynb);
* ✅ [`2022-12-19-Course-4-Planning-Exercises-Part-2.ipynb`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/Exercises/2022-12-19-Course-4-Planning-Exercises-Part-2.ipynb);
* ✅ [`2022-12-23-Course-4-Planning-Exercises-Part-3.ipynb`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/Exercises/2022-12-23-Course-4-Planning-Exercises-Part-3.ipynb).


### Projects
* ✅ 4.1: [Motion Planning and Decision Making for Autonomous Vehicles](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/4-Planning/4-1-Motion-Planning-Decision-Making).


### Exercises
* ✅ [4.1: Behavior Planning](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/4-Planning/Exercises/4-1-Behavior-Planning);
* ✅ [4.2: Trajectory Generation](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/4-Planning/Exercises/4-2-Trajectory-Generation);
* ✅ [4.3: Motion Planning](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/4-Planning/Exercises/4-3-Motion-Planning/);
* ✅ [4.4: Prediction](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/4-Planning/Exercises/4-4-Prediction/).


### Course Contents
The following topics are covered in course exercises:
* Behaviour Planning;
* Vehicle and environment states;
* Finite state machines (FSM);
* State transition functions;
* Cost functions for the self-driving car;
* 1D vehicle kinematics;
* Motion planning algorithms (Hybrid A*);
* Heuristics (non-holonomic / holonomic-with-obstacles);  
* Path planning with polynomial splines;
* Jerk minimising trajectories;
* Trajectory generation with velocity profile generators;
* Collision detection;
* Motion planning in CARLA Simulator;
* Multi-modal estimation algorithms;
* Model-based prediction;
* Data-driven prediction models;
* Intent / trajectory classification;
* ML classifiers;
* And so much more...

Other topics covered in course lectures and reading material:
* Motion planner scheduling / cycle updates;
* Finite-state machines;
* State spaces for self-driving cars;
* Cost and objective functions for self-driving cars;
* Feasability / safety / legality / comfort / efficiency trade-offs;
* Schedulers / messaging protocols;
* And so much more...


### Learning Outcomes
#### Lesson 1: Behavior Planning
* Design high-level behaviour planning systems for the self-driving car;
* Use a [Finite-state machine](https://en.wikipedia.org/wiki/Finite-state_machine) to model vehicle / environment states and transitions;
* Design weighted [cost functions](https://en.wikipedia.org/wiki/Loss_function) in C++;
* Implement a behaviour planner in C++.

#### Lesson 2: Trajectory Generation
* Use the C++ [`Eigen`](https://en.wikipedia.org/wiki/Eigen_\(C%2B%2B_library\)) library to design candidate trajectories in C++;
* Design and implement the [A*](https://en.wikipedia.org/wiki/A*_search_algorithm) and Hybrid [A*](https://en.wikipedia.org/wiki/A*_search_algorithm) search algorithms in C++;
* Perform trajectory matching and structured trajectory generation;
* Implement the quintic and polynomial solvers in C++;
* Study state-of-the-art path planning nets (e.g., [DeepTraffic](https://arxiv.org/abs/1801.02805)).

#### Lesson 3: Motion Planning
* Familiarise yourself with direct trajectory generation and path / velocity planning;
* Implement polynomial splines / sprials problem using Simpson's Rule in C++;
* Implement offset goals / boundary conditions in C++;
* Perform numerical approximation using discretisation;
* Design and implement velocity profiles in C++;
* Design and implement circle-based collision detection;
* Write and evaluate motion planning algorithms for the vehicle in the CARLA Simulator.

#### Lesson 4: Prediction
* Understand model-based and data-driven approaches to prediction;
* Learn about multi-modal estimators / trajectory clustering algorithms;
* Implement a hybrid-based approach using the Gaussian Naïve Bayes (GNB) classifier in C++; 
* Apply the GNB classifier to the vehicle trajectory prediction problem;
* Formulate process models for lane following (e.g., linear / non-linear point models, dynamic bicycle / non-holonomic kinematic models);
* Design a data-driven model to collect / process and predict vehicle trajectories.



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
* N/A.
