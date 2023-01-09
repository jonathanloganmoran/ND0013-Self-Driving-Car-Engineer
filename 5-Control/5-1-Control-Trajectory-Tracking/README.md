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
* ✅ Initialise a PID controller instance;
* ✅ Compute the PID error terms for the given cross-track error;
* ✅ Evaluate the expression of the PID controller using the updated terms;
* ✅ Create a function to update / modify the time-delta variable.
* ✅ Compile and run the programme to verify the ego-vehicle is stationary (controller not yet implemented).

### Use PID Controller for Vehicle Throttle ([`main.cpp`]())
* ✅ Implement the PID controller for throttle commands;
* ✅ Initialise the PID controller parameters with experimental gain values;
* ✅ Examine the performance of the PID controller using the experimental gain values;
* ✅ Tune the PID controller parameters (gain values) until trajectory is satisfactory.

### Use PID Controller for Vehicle Steering ([`main.cpp`]())
* ✅ Implement the PID controller for steering commands;
* ✅ Initialise the PID controller parameters with experimental gain values;
* ✅ Examine the performance of the PID controller using the experimental gain values;
* ✅ Tune the PID controller parameters (gain values) until trajectory is satisfactory.

### Evaluate the PID Controller Efficiency ([`main.cpp`]() and [`plot_pid.py`]())
* ✅ Save the throttle / steering command values into text files;
* ✅ Compute the steering / throttle error; 
* ✅ Plot the saved values using the `plot_pid.py` script and comment on the results;
* ⬜️ Answer the discussion questions in `answers.txt`.

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
In this project we use the C++ and Python APIs for the [CARLA Simulator](http://carla.org), as well as an open-source C++ solver, to design and test a PID controller for use in vehicle trajectory tracking. The starter code for this assignment is available at the Udacity GitHub repository [here](https://github.com/udacity/nd013-c6-control-starter/tree/master). 

#### Results

<img src="figures/2023-01-08-Figure-2-PID-Controller-with-Manual-Tuning-No-Collisions.gif" width="85%" height="85%" alt="PID Controller with acceptable PID gain values. Results in a tracked trajectory which avoid collisions and manoeuvres the vehicle to the stop sign at the end of the two-lane road.">

Results of the PID controller using manually-tuned gain values. Note that the movement of the ego-vehicle is quite erratic. This is due in part to non-optimal configuration of the gain values of the PID controller. This is due to the [limitation of the PID controller](https://en.wikipedia.org/wiki/PID_controller#Limitations_of_PID_control), which is a [model-free](https://en.wikipedia.org/wiki/Model-free_\(reinforcement_learning\)) algorithm whose parameters are tuned explicitly via [trial-and-error](https://en.wikipedia.org/wiki/Trial_and_error).

#### Prerequisites
In order to make use of this project, you must have the following dependencies installed —

Python:
* Python 3.7;
* [Carla Simulator](https://github.com/carla-simulator/carla)
* [PyPI Pip](https://pip.pypa.io/en/stable/installation/);
* [Numpy](https://numpy.org/);
* [Pygame](https://www.pygame.org/);
* [Gtest](https://pypi.org/project/gtest/);
* [Matplotlib]();
* [Pandas]().

C++:
* [C++14](https://en.wikipedia.org/wiki/C%2B%2B14)
* [Eigen 3.3.7](https://gitlab.com/libeigen/eigen/-/releases/3.3.7);
* [Carla Simulator](https://github.com/carla-simulator/carla);
* [`rpclib` 2.2.1](https://github.com/rpclib/rpclib).


#### Running and compiling the programme

##### Setting the hyperparameters

In this project we focus on the parameters of the PID controller. That is, the proportional-, integral-, and derivative-gain values used in the PID controller expression. These values can be manually configured in the [`main.cpp`]() file on lines 370-400. Note that the input arguments to the `init_controller` function from the [`pid_controller.cpp`]() file have the following ordering:

```cpp
PID::init_controller(k_p, k_i, k_d, lim_max_output, lim_min_output)
```

where `k_p`, `k_i`, `k_d` are the proportional-, integral- and derivative-gain values, respectively. The `lim_max_output`, `lim_min_output` values are the minimum and maximum allowed values for the respective controller. Any controller response values above these thresholds will be clipped to either the minimum or maximum values set here. 

After many experiment runs, I settled on the following set of gain values for both the steering- and the throttle-output PID controllers:

```cpp
pid_throttle.init_controller(0.21, 0.0006, 0.080, 1.0, -1.0);
pid_steer.init_controller(0.3, 0.0025, 0.17, 0.60, -0.60);
```

The parameter values chosen here were based loosely on the recommendations from an Udacity Mentor (link [here](https://knowledge.udacity.com/questions/939702)). After much trial-and-error, and without a specific tuning strategy, these were selected because they resulted in a controller implementation that was able to manoeuvre the vehicle to the desired location _without_ incurring any collision(s).

One other thing to note from the final parameters above is the use of a modified minimum- and maximum-allowed steering angle command value. Originally suggested was a steering [`lim_max_output`, `lim_min_output`] of [`1.2`, `-1.2`]. However, in the final experiment run, I change the steering range limits to be [`0.6`, `-0.6`] radians. This modification was proposed by Marcus Vinicius ([@marcusvinicius178](https://github.com/marcusvinicius178/) on GitHub) in [this Knowledge forum post](https://knowledge.udacity.com/questions/941131). Marcus' justification for this change is that a vehicle's [maximum turning radius] is usually in the range $[\pm35^{\circ}, \pm45^{\circ}]$ [1], which is much shorter than the original $\pm 1.2 \mathrm{rad} \sim 68.75^{\circ}$ proposed by Udacity. The vehicle minimum / maximum throttle response is, however, left unchanged at [`1.0`, `-1.0`] radians.

There are, of course, more intelligent ways of tuning a PID controller (e.g., [Ziegler-Nichols](https://en.wikipedia.org/wiki/Ziegler–Nichols_method) method) which are not explored here. In the previous lesson, we explored the use of the Twiddle local hill-climbing "coordinate ascent" optimisation algorithm in order to derive the optimal PID controller parameters (link [here](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/5-Control/Exercises/2023-01-03-Course-5-Control-Exercises-Part-1.ipynb)). For now, this is beyond the scope of this project.

Lastly, we suggest to the reader a manual tuning method which is described as follows:
1. Begin the PID controller tuning process by setting all gain values to zero;
2. Next, increase the proportional-gain value until the response is as close to the desired as possible;
   * At this point, oscillations or overshoot may occur. Additionally, the steady-state error may be non-zero;
3. To correct the steady-state error, adjust the integral-gain value;
   * If increasing the integral-gain also increases oscillations / overshoot, proceed to the following step;
4. To reduce overshoot / oscillations, adjust the derivative-gain value.


##### Creating the executable
In order to create the executable for this project, we follow similar instructions to that of [Project 4.1](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/4-Planning/4-1-Motion-Planning-Decision-Making). In summary, you must build the project from inside the `pid_controller` sub-folder using `CMAKE` as follows:

```console
root@foobar:/../5-1-Control-Trajectory-Tracking/project/pid_controller/#  cmake .
root@foobar:/../5-1-Control-Trajectory-Tracking/project/pid_controller/#  make .
```

Note that if using the Udacity Workspace VM, read and perform the following steps before attempting to build the project with the commands above.

##### Configuring the runtime environment
If you are using the Udacity VM, i.e., the project workspace running Ubuntu 18.04.5 LTS, you will need to perform several extra steps before the executable can be run.

First, make sure you have all project dependencies installed. Note that you must have the following C++ dependencies stored within the `project/pid_controller` folder:
* [`eigen-3.3.7`]();
* [`libcarla-install`]();
* [`rpclib`]().

These can be obtained directly from the [`nd013-c6-control-starter` repository](https://github.com/udacity/nd013-c6-control-starter). However, the `rpclib` files should be cloned directly from their source repository using the following commands while inside the `project/pid_controller` sub-directory:

```console
root@foobar:/../5-1-Control-Trajectory-Tracking/project/pid_controller/#  git clone https://github.com/rpclib/rpclib.git
```

Once the dependencies are installed and placed inside the `pid_controller` sub-directory, run the `CMAKE` script in the previous section to create the executable.

The second step you must perform is needed to successfully run the CARLA Simulator. In a new console window, set the superuser from `root` to `student` with the following command:

```console 
root@foobar:/opt/web-terminal/#  su - student
```

You may get a `Permission denied` error, but you can ignore this if you see the `student` user in the console command line as follows:

```console
student@foobar: ...
```

Now, with the `student` user account configured, navigate to the `/opt/carla-simulator` sub-directory and run the build script as follows:

```console
student@foobar:  cd /opt/carla-simulator
student@foobar:/opt/carla-simulator/#  SDL_VIDEODRIVER=offscreen ./CarlaUE4.sh -opengl
```

This should set the CARLA Simulator to headless mode and prevent the programme from incurring any `Segmentation fault (core dumped)` errors.

The third step is to, in a new console window (could be the first you ran the project `CMAKE` script in — just make sure you've got the `root` user active), run the [`install-ubuntu.sh`]() build script. This will install the necessary dependencies for CARLA specific to the Ubuntu LTS runtime environment:

```console
root@foobar:/opt/web-terminal/5-1-Control-Trajectory-Tracking/project/#  ./install-ubuntu.sh
```

You may have to agree to the installation of the dependencies by entering the "`Y`" key when prompted.

##### Executing the programme
With the project dependencies installed and the Ubuntu VM configured, run the programme executable with the following:

```console
root@foobar:/opt/web-terminal/5-1-Control-Trajectory-Tracking/project/#  ./run_main_pid.sh
```

Make sure you also have the CARLA configuration script running simultaneously in a separate window (see above).

Depending on the environment you are running, you may experience a silet fail error. On the Udacity VM, this is expected. Simply use the CTRL + C keys to halt the programme. Run the programme script again using the same commands above. The programme should open in a new Pygame window without further problems. If you continue to experience issues running the [`run_main_pid.sh`]() script, make sure that the script is being run from a _separate_ console window with the `root` superuser **and** that you have the CARLA headless script executing under the `student` superuser in another console window (see _Configuring the runtime environment_ for instructions).

With the programme running successfully, you should observe the ego-vehicle manoeuvre automatically about the environment. Depending on the PID controller parameter values you set in [`main.cpp`](), the ego-vehicle may avoid collisions and execute steer / throttle commands which allow it to arrive succesfully at the stop sign near the end of the two-lane road. In experiment runs where this is **not** the case, you will quickly notice the ego-vehicle exhibit erratic steering / throttle actuations, and possibly experience one or more collisions with its surroundings, as such:

<img src="figures/2023-01-08-Figure-1-PID-Controller-with-Manual-Tuning-Collisions.gif" width="85%" height="85%" alt="PID Controller with poorly configured PID gain values. Results in a collision with one or more obstacles in the environment.">

##### Evaluating the controller results
In order to evaluate the performance of the controller, run the provided [`plot_pid.py`]() script after the PID controller is configured and the desired vehicle goal trajectory (i.e., the stop sign) has been reached. This Python script will plot the steering- and throttle-error over time using the values recorded in [`main.cpp`]().

```console
root@foobar:/opt/web-terminal/5-1-Control-Trajectory-Tracking/project/pid_controller/#  python3 plot_pid.py
```

If you get an error running the above command, make sure that you have both Matplotlib and Pandas dependencies installed. Otherwise, run:

```console
root@foobar:/../#  pip3 install matplotlib
root@foobar:/../#  pip3 install pandas 
```

#### Discussion
After running the [`plot_pid.py`]() script, you will get plots that look similar to the following:

<img src="figures/2023-01-08-Figure-3-PID-Controller-Error-Plot-Steering.png" height="85%" width="85%" alt="The error rate of the steering response of the PID controller using the final manually-tuned gain values.">

In the first figure — the plot of the steering command data, the ego-vehicle exhibits sharp steering commands. This is observed in the plot as the sharp oscillations (high peaks / low valleys). Demonstrated is the back-and-forth (erratic) steering of the ego-vehicle performed in order to avoid collision. The PID controller design choice to clip the minimum / maximum steering allowed from $\pm 1.2 \mathrm{rad}$ to $\pm 0.6 \mathrm{rad}$ does not seem to prevent extremely sharp transitions between left- and right- angled steering commands, nor does it seem like the final controller gain values selected here help minimise the overall steering error experienced by the ego-vehicle with respect to the reference trajectory (the "best" polynomial spiral selected for each manoeuvre). 

The choice of PID controller gain values for the steering response were selected such that the largest penalty (weight) was given to the proportional- term, since we wanted to keep the ego-vehicle roughly aligned with the curvature of the intended goal trajectory in order to avoid obstacles in the expected amount of time. The next-largest weight was given to the derivative- term in order to keep the steering response values within reasonable limits (i.e., penalise extremely abrupt changes to the steering angle). The least amount of weight was given to the integral- term, which attempts to penalise the cumulative uncorrected steering error in a consecutive time interval. This might be one area of weakness in the current implementation, where a large cumulative error w.r.t. steering might be apparent. In other words, if we decided to penalise larger cumulative errors more, we might notice a smoother steering trajectory, rather than just penalising large derivative errors (i.e., sharp changes in steering angle).        


<img src="figures/2023-01-08-Figure-4-PID-Controller-Error-Plot-Throttle.png" height="85%" width="85%" alt="The error rate of the throttle response of the PID controller using the final manually-tuned gain values.">

In the second figure — the plot of the throttle command data, the controller error seems to stabilise to a [steady-state](https://en.wikipedia.org/wiki/Steady_state) after the initial [rise time](https://en.wikipedia.org/wiki/Rise_time) (as indicated by the sharp oscillations in the early part of the experiment run). While the controller maintains a relatively large magnitude of error throughout the experiment run, we notice a gradual downward trend of the throttle error (blue) towards the centre-line (red) at $y = 0$. This leads one to believe from the overall throttle response that this PID controller is more effectively tuned than the steering angle controller above.

The parameters of the PID controller for the throttle commands were selected such that the largest weight was given to the proportional- term. This was to ensure that the throttle response was no greater than the intended throttle from the reference trajectory (i.e., a larger proportional-gain value means a greater penalty for _actual_ throttle commands that differ greatly from the _expected_). The next-largest gain value was chosen for the derivative- term, which was set such that the change in throttle was minimal. This was designed to give a more comfortable ride experience for the passenger(s) while keeping vehicle acceleration within theoretical / feasible limits. Lastly, the gain for the integral- term was given the least weight since the expected acceleration or braking were assumed to happen in relatively short intervals. With this assumption, there would be less chance of accumulating a large error over a great period of time $\Delta t$ which yields a less-significant contribution of error from this integral- term. 

When taking into consideration both the CARLA simulation runs and the above error plots, we emphasise the importance of PID controller parameter selection. Since the PID controller is a type of [model-free](https://en.wikipedia.org/wiki/Model-free_\(reinforcement_learning\)) algorithm, and because we selected the gain values without intuition via a [trial-and-error](https://en.wikipedia.org/wiki/Trial_and_error) process, we are hesitant to recommend the implementation of the PID controller in its current state for use with this task. In regards to related literature, there have been experiments which successfully utilise the PID controller to perform complex trajectory tracking. In Farag 2019 [2], only the cross-track error (CTE) is used as input to the PID controller (as done here). However, the Farag paper proposes an ad-hoc tuning method with a process more-intuitive than merely adjusting parameters by-hand until a desired behaviour is observed. 

Lastly, other tuning methods exist both in literature and in practise — such as the [Ziegler-Nichols](https://en.wikipedia.org/wiki/Ziegler–Nichols_method) method or the [Twiddle](https://martin-thoma.com/twiddle/) parameter optimisation algorithm, but are not considered in this project due to their complexity with respect to their integration into the project codebase. While [model-free](https://en.wikipedia.org/wiki/Model-free_\(reinforcement_learning\)) algorithms such as the [PID controller](https://en.wikipedia.org/wiki/PID_controller) are often straight-forward and widely-used approaches to control, model-based methods such as [model predictive control](https://en.wikipedia.org/wiki/Model_predictive_control) (MPC) are more-robust and rely on dynamic process models which can anticipate future events and choose control actions in response.

## 3. Closing Remarks
###### Alternatives
* Implement a model-based control method (e.g., [Nonlinear MPC](https://en.wikipedia.org/wiki/Model_predictive_control#Nonlinear_MPC)).

##### Extensions of task
* Experiment with P-, PD- and PID controller gains to determine their individual contribution to the overall error;
* Use other tuning methods to derive the optimal PID controller parameters (e.g., [Twiddle](https://martin-thoma.com/twiddle/)).

## 4. Future Work
* ⬜️ Experiment with P-, PD- and PID controller gains to determine their individual contribution to the overall error.

## Credits
This assignment was prepared by Sebastian Thrun, David Siller, Andrew Gray et al., 2021 (link [here](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd0013)).

References
* [1] Berjoza, D. Research in Kinematics of Turn For Vehicles and Semitrailers. Proceedings of the 7th International Scientific Conference - Engineering for Rural Development. 7(1):177-182. 2008. [ISSN:1961-3043](https://www.tf.llu.lv/conference/proceedings2008/Papers/34_Berjoza.pdf).
* [2] Farag, W. A. Complex Trajectory Tracking Using PID Control for Autonomous Driving. International Journal of Intelligent Transportation Systems Research. 18(2):
356-366. 2020. [doi:10.1007/s13177-019-00204-2](https://doi.org/10.1007/s13177-019-00204-2).


Helpful resources:
* [`nd013-c6-control-starter` by @udacity | GitHub](https://github.com/udacity/nd013-c6-control-starter/tree/master).