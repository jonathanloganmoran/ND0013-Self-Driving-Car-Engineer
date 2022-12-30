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
### Behaviour Planning ([`behavior_planner_FSM.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner/behavior_planner_FSM.cpp))
* ✅ Implement the look-ahead distance function;
* ✅ Compute a goal state (pose, velocity) situated behind a stopping point;
* ✅ Compute the goal speed for a nominal state;
* ✅ Track the existing goal when in `DECEL_TO_STOP` state;
* ✅ Calculate the distance travelled w.r.t. rectlinear motion;
* ✅ Compute `DECEL_TO_STOP` state w.r.t. distance rather than speed;
* ✅ Move the ego-vehicle to a `STOPPED` state;
* ✅ Track the existing goal when in `STOPPED` state;
* ✅ Move ego-vehicle to a `FOLLOW_LANE` state.

### Cost Functions ([`cost_functions.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner/cost_functions.cpp))
* ✅ Evaluate cost w.r.t. placement of objects (as circles);
* ✅ Evaluate cost w.r.t. distance to objects (as circles);
* ✅ Evaluate cost w.r.t. distance between last waypoint on spiral and final goal state.

### Motion Planning ([`motion_planner.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner/motion_planner.cpp))
* ✅ Plan paths based on perpendicular heading direction;
* ✅ Plan paths based on goal-offset location.

### Velocity Profile Generation ([`velocity_profile_generator.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner/velocity_profile_generator.cpp))
* ✅ Compute the distance travelled w.r.t. rectilinear motion for the linear velocity profile;
* ✅ Compute the final velocity w.r.t. rectilinear motion for the linear velocity profile;

### Fine-Tuning Hierarchial Planner ([`planning_params.h`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner/planning_params.h))
* ✅ Define the number of paths (goal states) to enumerate;
* ✅ Define the number of waypoints to enumerate for each path spiral.


## 1. Introduction
In this project we implement several core components of the traditional [hierarchical task network](https://en.wikipedia.org/wiki/Hierarchical_task_network)-based planner [1]: the behaviour planner and the motion planner. Both the behaviour and the motion planner modules will be able to:
1. Avoid static objects located on the sides of the road. These obstacles (e.g., cars, bicycles, trucks) are placed such that their location intersects the lane travelled by the ego-vehicle;
    * The ego-vehicle must avoid crashing into these obstacles by executing a proper "nudge" or lane-change manoeuvre;
2. Handle intersections of arbitrary type (e.g., 3-way / 4-way intersections, round-abouts); 
    * The ego-vehicle must come to a complete stop at any of these intersections;
3. Track the lane centre line.

Many behaviour / motion planning systems for autonomous vehicles utilise [finite-state machine](https://en.wikipedia.org/wiki/Finite-state_machine), such as Stanford's Junior [], the DARPA Grand Challenge Winner. In this project we also make use of the finite-state machine to handle state transitions from one active manoeuvre (i.e., `DECEL_TO_STOP`) to another. The FSM implements the behaviour planning module of the hierarchical-task network. This submodule manages the goal-state of the ego-vehicle and lookahead distance functionality (i.e., the longitudinal distance needed to travel by the ego-vehicle in order to come to a complete stop). Here we introduce comfort constraints which ensure that the deceleration for     

## 2. Programming Task
### 2.1. Motion Planning and Decision Making
#### Background
The vehicle behaviours we consider for this project are:
* `FOLLOW_LANE`: Ego-vehicle keeps lane and maintains the configured `_speed_limit`;
* `FOLLOW_VEHICLE`: Ego-vehicle keeps lane and maintains the speed of the lead vehicle;
* `DECEL_TO_STOP`: Ego-vehicle moves towards the goal-state location (minus a distance `_stop_line_buffer`) in order to come to a complete stop using a comfortable acceleration;
* `STOPPED`: Ego-vehicle waits at e.g., intersection / stop sign for `_req_stop_time` (sec).

In the current implementation of the behaviour planner (in [`behavior_planner_FSM.cpp`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner/behavior_planner_FSM.cpp)) we neglect the `FOLLOW_VEHICLE` action and instead choose to move the vehicle to a different lane to avoid any possible collisions.

#### Results

The following output was produced during a test run of the 
<img src="out/2022-12-29-Figure-1-Testing-Motion-Planner-in-CARLA.gif" width="90%" height="90%" alt="Figure 1. Testing the Motion Planner in the CARLA Simulator.">

$$
\begin{align}
\textrm{Figure 1. Testing the Motion Planner in the CARLA Simulator.}
\end{align}
$$

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

While we have chosen to omit these dependencies from this repository, you may find the archived copies on the starter code repository [here](https://github.com/udacity/nd013-c5-planning-starter/tree/master/project).

#### Running and compiling the programme

##### Setting the hyperparameters
The motion planner has several hyperparameters whose values can be modified. Inside the [`planning_params.h`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner/planning_params.h) file, you will find the following two parameters:

```cpp
/*** Planning Constants ***/
// Number of paths (goals) to create
// CANDO: Modify the number of path offsets to generate
#define P_NUM_PATHS 5                  // Number of deviations to enumerate
``` 

and

```cpp
// Number of waypoints to use in polynomial spiral (path)
// CANDO: Modify the number of waypoints to use in each path
// NOTE: Needs to be sufficiently large to avoid compile error
#define P_NUM_POINTS_IN_SPIRAL 25
```

which are on lines 19 and 51, respectively.

Modifying these values will allow you to control the number of and the coarseness / fineness of the polynomial spiral-based paths. 

We recommended setting the `P_NUM_POINTS_IN_SPIRAL` to a value of `25` or greater in order to avoid runtime errors (`Unknown PCM default` / `segmentation fault (core dumped)`).

##### Configuring CMAKE build
In order to use this modified version of the project code, which has renamed the `'project/starter_code/'` subfolder to [`'project/planner'`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner), you must change line 3 of the _original_ [`run_main.sh`](https://github.com/udacity/nd013-c5-planning-starter/blob/master/project/run_main.sh) script to the following:

```sh
./{SUBFOLDER_NAME}/spiral_planner&
```

where `{SUBFOLDER_NAME}` should be `planner`, which matches the renamed subfolder in this repository. Note that you may ignore this step if running the [`run_main.sh`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/run_main.sh) script from inside this repository.

##### Creating the executable
In order to create the executable file, open a new console window and navigate to the [`'/project/planner'`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner) subdirectory. Then, execute the following commands in the console:

```console
root@foobar:/opt/web-terminal/4-1-Motion-Planning-Decision-Making/project/planner/#  cmake .
root@foobar:/opt/web-terminal/4-1-Motion-Planning-Decision-Making/project/planner/#  make
```

Note that here our project root directory is named `4-1-Motion-Planning-Decision-Making`, but this might be different depending on how you obtained the [starter code](https://github.com/udacity/nd013-c5-planning-starter) for this project.

##### Configuring the runtime environment
If you are using the Udacity VM, i.e., the project workspace running Ubuntu 18.04.5 LTS, you will need to perform two extra steps before the executable can be run.

The first step is to run the CARLA configuration script. First, set the superuser from `root` to `student` in a separate console window with the following command:
```console
root@foobar:/opt/web-terminal/#  su - student
```

You may get a `Permission denied` error, but you can ignore this if you see the `student` user in the console command line as follows:

```console
student@foobar:  ...
```

Now, with the `student` user account configured, navigate to the `/opt/carla-simulator/` subdirectory and run the build script, i.e.,

```console
student@foobar:/#  cd /opt/carla-simulator
student@foobar:/opt/carla-simulator/#  SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl
```

This should set the CARLA Simulator to headless mode and prevent the programme from incurring any `Segmentation fault (core dumped)` errors.

The second step is to run the [`install-ubuntu.sh`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/install-ubuntu.sh) build script which installs the necessary dependencies for the Ubuntu LTS runtime environment. To do so, open a new console window, navigate to the project subfolder (here, that's `4-1-Motion-Planning-Decision-Making/project`), and run the following:

```console
root@foobar:/opt/web-terminal/4-1-Motion-Planning-Decision-Making/project/#  ./install-ubuntu.sh
```

You may have to agree to the installation of the dependencies by entering `Y` when prompted.

With these runtime configuration steps out of the way, open a _separate console window_ (making sure the CARLA build script from earlier is running in another). Then, navigate back to the project root directory:

```console
root@foobar:/opt/web-terminal/#  cd /opt/web-terminal/4-1-Motion-Planning-Decision-Making
```

and proceed to the programme execution steps in the following section.

##### Executing the programme
Once the project has been built successfully, the executable can be run with the following command:
```console
root@foobar:/opt/web-terminal/4-1-Motion-Planning-Decision-Making/project/#  ./run_main.sh
```

Depending on which environment you are running, you may experience a silent fail error. On the Udacity VM, this is expected. Simply use CTRL + C keys to halt the programme. Run the programme script again (same as above), then the CARLA Simulator should start without problems. If you continue to experience errors running the [`./run_main.sh`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/run_main.sh) script, make sure that you're running the script in a _new_ console window and have the CARLA headless script running under the `student` user in another console window (see _Configuring the runtime environment_). 

In the case that you get a `Unknown PCL default` error, try modifying the `P_NUM_POINTS_IN_SPIRAL` hyperparameter value (increasing this number worked for me here).

With the programme running successfully, you should observe the ego-vehicle manoeuvre automatically around obstacles, come to a complete stop at the stop sign, and even be able to make a right-turn from the complete stop into one of the available lanes.

##### More information
This build / configure / execute sequence has been adapted from the [original `how_to_run.txt` instructions](https://github.com/udacity/nd013-c5-planning-starter/blob/master/project/how_to_run.txt). For instructions pertaining to this repository for use with the Udacity VM, see [`how_to_run.txt`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/how_to_run.txt) file here. 

This concludes the programming tasks for this project. Refer to the "Tasks" check-list to make sure all TODOs are completed. Recommended literature to review: hierarchical planning [1], finite-state machines for AVs [2][5], real-time motion planning [6], path planning algorithms [3], and applications to vehicles [2][6][7].


## 3. Closing Remarks
###### Alternatives
* Increase the number of points in the path (`P_NUM_POINTS_IN_SPIRAL`);
* Change the number of paths to generate (`P_NUM_PATHS`);

##### Extensions of task
* Implement a motion controller for the `DECEL_TO_STOP` state in [`state_transition`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner/behavior_planner_FSM.cpp) function;
* Implement the `FOLLOW_VEHICLE` state in [`state_transition`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner/behavior_planner_FSM.cpp) function;
* Choose a trajectory planner suitable for real-time applications (e.g., [Frenet Optimal Trajectory](https://fjp.at/posts/optimal-frenet/) [6]);
* Consider using deep learning-based techniques for path planning / obstacle avoidance (e.g., RL with particle filtering [4])

## 4. Future Work
* ⬜️ Implement a motion controller for the `DECEL_TO_STOP` state in [`state_transition`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/4-Planning/4-1-Motion-Planning-Decision-Making/project/planner/behavior_planner_FSM.cpp) function.

## Credits
This assignment was prepared by Munir Jojo-Verge, Aaron Brown et al., 2021 (link [here](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd0013)).

References
* [1] Orzechowski, P. F. et al. Decision-Making for Automated Vehicles Using a Hierarchical Behavior-Based Arbitration Scheme. IEEE Intelligent Vehicles Symposium (IV). pp. 767-774. 2020. [doi:10.1109/IV47402.2020.9304723](https://doi.org/10.1109/IV47402.2020.9304723).
* [2] Montemerlo, M. et al. Junior: The Stanford Entry in the Urban Challenge. Journal of Field Robotics. 2008. 25(9):569-597. [doi:10.1002/rob.20258](https://doi.org/10.1002/rob.20258).
* [3] LaValle, S. M. Combinatorial Motion Planning. In: Planning Algorithms. Cambridge University Press. 2006. pp.206-256. [doi:10.1017/CBO9780511546877.008]().ISBN-13: 978-0521862059.
* [4] Everett, M. et al. Motion Planning Among Dynamic, Decision-Making Agents with Deep Reinforcement Learning. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). 2018. [doi:10.1109/IROS.2018.8593871](https://doi.org/10.1109/IROS.2018.8593871).
* [5] Dolgov, D. et al. Practical Search Techniques in Path Planning for Autonomous Driving. Association for the Advancement of Artificial Intelligence. Pre-print. 2008. [https://www.aaai.org/Library/Workshops/2008/ws08-10-006.php](https://www.aaai.org/Papers/Workshops/2008/WS-08-10/WS08-10-006.pdf).
* [6] Xu, W. et al. A Real-Time Motion Planner with Trajectory Optimization for Autonomous Vehicles. IEEE International Conference on Robotics and Automation. pp.2061–2067. 2012. [doi:10.1109/ICRA.2012.6225063](https://doi.org/10.1109/ICRA.2012.6225063);
* [7] Yu, L. A Driving Behavior Planning and Trajectory Generation Method for Autonomous Electric Bus. Future Internet 2018, 10, 51. [doi:10.3390/fi10060051](https://doi.org/10.3390/fi10060051);
* [8] Liang, T. C. et al. Practical and flexible path planning for car-like mobile robot using maximal-curvature cubic spiral. Robotics and Autonomous Systems. Elsevier. 52(4):312-335. 2005. [doi:10.1016/j.robot.2005.05.001](https://doi.org/10.1016/j.robot.2005.05.001).

Helpful resources:
* [ND0013: C5 Planning Starter | GitHub](https://github.com/udacity/nd013-c5-planning-starter);
* [Research Projects in Decision-Making and Motion Planning | Institute of Measurement and Control Systems (MRT) at KIT](https://www.mrt.kit.edu/english/Decision-Making_and_Motion_Planning.php);
* [Planning Algorithms by LaValle, S. M. | Cambridge University Press (2006)](doi:10.1017/CBO9780511546877.008);
* [Planning and Decision Making for Aerial Robots by Sebbane, Y. S. | Springer (2014)](https://doi.org/10.1007/978-3-319-03707-3);
* [Dynamic Deadlines in Motion Planning for Autonomous Driving Systems by E. Fang | Lit. Review of AV Planning systems](https://www.semanticscholar.org/paper/Dynamic-Deadlines-in-Motion-Planning-for-Autonomous-Fang/ad76971b987f586078235c18ea68d2210d848f08).