# -----------------------------------------------------------------------------
# Lesson "5.1: PID Control"
# Authors     : Sebastian Thrun.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file: Implements the Proportional-Integral-Derivative (PID)
#                       Controller with hyperparameter tuning using the
#                       Twiddle algorithm. The programme simulates robot motion
#                       along a horizontal reference trajectory and compares
#                       the accuracy of each controller type (P-, PD-, PID).
# -----------------------------------------------------------------------------

import matplotlib as mpl
# Setting the default font to use with Matplotlib
mpl.rc('font', family='Times New Roman')
import matplotlib.pyplot as plt
import numpy as np
import random
from typing import List, Tuple


class Robot(object):
    '''The simulated Robot class.
    
    The robot moves about the 2D environment given a position (x, y) and
    heading (`orientation`) angle, i.e., direction of motion.

    :param x: Position of the robot along the $x$-axis.
    :param y: Position of the robot along the $y$-axis.
    :param orientation: Heading angle of the robot, i.e.,
        the direction parameter $\theta$ used in the bicycle motion model.
    :param length: Length of the robot, i.e.,
        the parameter $L$ used in the bicycle motion model.
    :param steering_noise: Steering measurement noise sampled from a
        Gaussian normal distribution.
    :param distance_noise: Distance measurement noise sampled from a
        Gaussian normal distribution.
    :param steering_drift: Perturbation of the steering angle sampled from a
        Gaussian normal distribution.
    '''

    def __init__(self,
            length: float=20.0
    ):
        """Creates a `Robot` instance.

        Initialises the robot 2D location and orientation angle to the
        position along the origin (0, 0) with a heading of `0.0`.

        :param length: Desired length to initialise the robot with. 
        """

        self.x = 0.0
        self.y = 0.0
        self.orientation = 0.0
        self.length = length
        self.steering_noise = 0.0
        self.distance_noise = 0.0
        self.steering_drift = 0.0

    def set(self, 
            x: float, 
            y: float,
            orientation: float
    ):
        """Sets the robot's 2D location and orientation to the given values.
        
        :param x: Position along the $x$-axis to assign to the robot.
        :param y: Position along the $y$-axis to assign to the robot.
        :param orientation: Heading angle to assign to the robot. 
        """

        self.x = x
        self.y = y
        self.orientation = orientation % (2.0 * np.pi)

    def set_noise(self, 
            steering_noise: float, 
            distance_noise: float
    ):
        """Sets the steering- and measurement- noise parameters.

        This update function allows for the use of dynamic noise values,
        which is often useful in particle filters.

        :param steering_noise: Noise value to use for the steering measurement.
        :param distance_noise: Noise value to use for the distance measurement.
        """

        self.steering_noise = steering_noise
        self.distance_noise = distance_noise

    def set_steering_drift(self, 
            drift: float
    ):
        """Sets the systematical steering drift parameter.

        :param drift: Drift value to use for the steering angle.
        """

        self.steering_drift = drift

    def move(self, 
            steering: float, 
            distance: float, 
            tolerance: float=0.001, 
            max_steering_angle: float=np.pi / 4.0
    ):
        """Move the robot to the next time-step with the provided controls.

        :param steering: Front-wheel steering angle $\delta$.
        :param distance: Total distance travelled by vehicle in this time-step,
            must be non-negative.
        :param tolerance: Tolerance value for the manoeuvre,
            values outside this threshold indicate a turning manoeuvre.
        :param max_steering angle: Maximum front-wheel steering angle.
        """

        # Thresholding of input steering angle and distance values 
        if steering > max_steering_angle:
            steering = max_steering_angle
        if steering < -max_steering_angle:
            steering = -max_steering_angle
        if distance < 0.0:
            distance = 0.0
        # Computing the noise values
        steering2 = random.gauss(steering, self.steering_noise)
        distance2 = random.gauss(distance, self.distance_noise)
        # Applying the steering drift parameter to the steering angle
        steering2 += self.steering_drift
        # Compute the radius of the turn / manoeuvre to execute
        turn_radius = np.tan(steering2) * distance2 / self.length
        if abs(turn_radius) < tolerance:
            # Compute the straight-line distance approximation of the manoeuvre
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn_radius) % (2.0 * np.pi)
        else:
            # Evaluate the approximate kinematic bicycle motion model
            # for the desired turning manoeuvre
            radius = distance2 / turn_radius
            # Compute the changing rate of $x$, $y$, $\delta$
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn_radius) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        """Overrides the default print function with vehicle state values."""
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


def make_robot(
) -> Robot:
    """Creates and initialises a `Robot` instance.

    Resets the robot back to the initial position with non-zero steering drift.
    
    :returns: A new, configured instance of the `Robot` class.
    """

    robot = Robot()
    robot.set(0, 1, 0)
    robot.set_steering_drift(10 / 180 * np.pi)
    return robot


def run_p(
        robot: Robot, 
        params: List[float], 
        n: int=100, 
        speed: float=1.0
) -> Tuple[List[float], List[float]]:
    """Simulates robot movement using proportional control.
    
    The robot's steering angle $\alpha$ is computed relative to the
    proportional gain controller, which follows the equation:
        $\alpha = -\tau * \mathrm{CTE}$,
    where the cross-track error $\mathrm{CTE}$ is defined as the
    robot position along the $y$-axis, assuming a horizontal reference
    trajectory about the $x$-axis. Here, the gain factor $\tau$ is proportional
    to the cross-track error $\mathrm{CTE}$.

    :param robot: `Robot` class instance representing the vehicle to manoeuvre. 
    :param params: Set of hyperparameter values to use in the P-controller,
        should be one value in order: $[\tau_{p}]$.
    :param n: Number of time-steps to simulate.
    :param speed: Velocity (m/s) at which to drive the vehicle.
    :returns: Set of x- and y-coordinates of the simulated trajectory.
    """

    ### Make sure all parameters were provided
    assert len(params) == 1
    tau_p = params[0]
    # The list of $x$- and $y$-values for the simulated trajectory
    x_trajectory = []
    y_trajectory = []
    # Simulate the robot movement across `n` time-steps
    for i in range(n):
        # Get the current cross-track error relative to reference trajectory
        cte = robot.y
        # Compute the steering angle w.r.t. the proportional gain controller
        steer = -tau_p * cte
        # Execute the steering command
        robot.move(steer, speed)
        # Append the updated robot position coordinates to the trajectory lists
        _x, _y = robot.x, robot.y
        x_trajectory.append(_x)
        y_trajectory.append(_y)
    return x_trajectory, y_trajectory


def run_pd(
        robot: Robot, 
        params: List[float],
        n: int=100, 
        speed: float=1.0
) -> Tuple[List[float], List[float]]:
    """Simulates robot movement using proportional-derivative control.

    The proportional-derivative controller used here follows the equation:
        $\alpha = -\tau_{p} * \mathrm{CTE} - \tau_{d} * \Delta \mathrm{CTE}$,
    where the steering input angle $\alpha$ is computed w.r.t. the proportional
    gain controller — i.e., the product of proportional gain $\tau_{p}$ (the
    "response strength") and the current cross-track error $\mathrm{CTE}_{t}$,
    and the derivative of the cross-track error $\Delta \mathrm{CTE}$, i.e., 
        $\frac{\mathrm{CTE}_{t} - \mathrm{CTE}_{t-1}}{\Delta t}$,
    scaled by a constant gain factor $\tau_{d}$. 
    
    Assumed here is a constant unit time-step $\Delta t = 1.0$ and a reference
    trajectory defined as a constant horizontal trajectory about the $x$-axis
    at $y=0$.
    
    The proportional-derivative controller implemented here is used to direct
    the robot motion towards the horizontal reference trajectory by giving
    steering angle commands computed w.r.t. both the normally-distributed
    steering drift and steering-, distance measurement noise.

    :param robot: `Robot` class instance representing the vehicle to manoeuvre. 
    :param params: Set of hyperparameter values to use in the PD-controller,
        should be two values in order: $[\tau_{p}$, \tau_{d}]$.
    :param n: Number of time-steps to simulate.
    :param speed: Velocity (m/s) at which to drive the vehicle.
    :returns: Set of x- and y-coordinates of the simulated trajectory.
    """

    ### Make sure all parameters were provided
    assert len(params) == 2
    tau_p, tau_d = params
    # The list of $x$- and $y$-values for the simulated trajectory
    x_trajectory = []
    y_trajectory = []
    # Set the constant unit time-step value (used in derivative term)
    delta_t = 1.0
    # Initialise the "previous" and current cross-track error values
    cte_prev = robot.y
    cte_curr = 0.0
    # Simulate the robot movement across `n` time-steps
    for i in range(n):
        # Get the current cross-track error relative to reference trajectory
        cte_curr = robot.y
        # Compute the steering angle w.r.t. proportional-derivative controller
        # NOTE: `cte_dot` is derivative of CTE w.r.t. constant unit time-step
        cte_dot = (cte_curr - cte_prev) / delta_t
        # Compute the steering angle w.r.t. proportional and derivative gain
        steer = -tau_p * cte_curr - tau_d * cte_dot
        # Set the "previous" CTE value to the current error before manoeuvre
        cte_prev = cte_curr
        # Execute the steering command computed with the PD-controller
        robot.move(steer, speed)
        # Append the updated robot position coordinates to the trajectory lists
        _x, _y = robot.x, robot.y
        x_trajectory.append(_x)
        y_trajectory.append(_y)
    return x_trajectory, y_trajectory


def run_pid(
        robot: Robot, 
        params: List[float],
        n: int=100, 
        speed: float=1.0
) -> Tuple[List[float], List[float]]:
    """Simulates robot movement using proportional-integral-derivative control.

    The proportional-integral-derivative controller used here follows:
    $$\begin{align}
    \alpha &= -\tau_{p} * \mathrm{CTE} 
              - \tau_{d} * \Delta \mathrm{CTE} 
              - \tau_{i} * \int_{0}^{t} \mathrm{CTE},
    \end{align}$$
    where the integral-term $\int_{0}^{t} \mathrm{CTE}$ is given as the sum of
    the instantaneous error over time. This gives the accumulated offset that
    should have been previously corrected.

    Assumed here is a constant unit time-step $\Delta t = 1.0$ and a reference
    trajectory defined as a constant horizontal trajectory about the $x$-axis
    at $y=0$.
    
    The proportional-integral-derivative controller implemented here is used
    to direct the robot motion towards the horizontal reference trajectory by
    giving steering angle commands computed w.r.t. both the normally-distributed
    steering drift and steering-, distance measurement noise.

    :param robot: `Robot` class instance representing the vehicle to manoeuvre.    
    :param params: Set of hyperparameter values to use in the PID-controller,
        should be three values in order: $[\tau_{p}, \tau_{d}, \tau_{i}]$.
    :param n: Number of time-steps to simulate.
    :param speed: Velocity (m/s) at which to drive the vehicle.
    :returns: Set of x- and y-coordinates of the simulated trajectory.
    """

    # Make sure all parameters were provided
    assert len(params) == 3
    tau_p, tau_d, tau_i = params
    # The list of $x$- and $y$-values for the simulated trajectory
    x_trajectory = []
    y_trajectory = []
    # The list of all previous cross-track errors (used in integral term)
    cte_values = []
    # Set the constant unit time-step value (used in derivative term)
    delta_t = 1.0
    # Initialise the "previous" and current cross-track error values
    cte_prev = robot.y
    cte_curr = 0.0
    # Simulate the robot movement across `n` time-steps
    for i in range(n):
        # Get the current cross-track error relative to reference trajectory
        cte_curr = robot.y
        cte_values.append(cte_curr)
        ### Compute the steering angle w.r.t. PID controller
        # First, caclulate the derivative term
        # NOTE: `cte_dot` is derivative of CTE w.r.t. constant unit time-step
        cte_dot = (cte_curr - cte_prev) / delta_t
        # Then, calculate the integral term 
        cte_int = np.sum(cte_values)
        # Form the expression of the complete PID controller
        # w.r.t. the proportional, integral, and derivative gain values
        steer = -tau_p * cte_curr - tau_d * cte_dot - tau_i * cte_int
        # Set the "previous" CTE value to the current error before manoeuvre
        cte_prev = cte_curr
        # Execute the steering command computed with the PID-controller
        robot.move(steer, speed)
        # Append the updated robot position coordinates to the trajectory lists
        _x, _y = robot.x, robot.y
        x_trajectory.append(_x)
        y_trajectory.append(_y)
    return x_trajectory, y_trajectory


def run_pid_tuned(
        robot, 
        params, 
        n=100, 
        speed=1.0
) -> Tuple[List[float], List[float], float]:
    """Simulates robot movement using PID-control with tuned parameters.
    
    The proportional-integral-derivative controller used here follows:
    $$\begin{align}
    \alpha &= -\tau_{p} * \mathrm{CTE} 
              - \tau_{d} * \Delta \mathrm{CTE} 
              - \tau_{i} * \int_{0}^{t} \mathrm{CTE},
    \end{align}$$
    where the integral-term $\int_{0}^{t} \mathrm{CTE}$ is given as the sum of
    the instantaneous error over time. This gives the accumulated offset that
    should have been previously corrected.

    Assumed here is a constant unit time-step $\Delta t = 1.0$ and a reference
    trajectory defined as a constant horizontal trajectory about the $x$-axis
    at $y=0$.
    
    The proportional-integral-derivative controller implemented here is used
    to direct the robot motion towards the horizontal reference trajectory by
    giving steering angle commands computed w.r.t. both the normally-distributed
    steering drift and steering-, distance measurement noise.

    :param robot: `Robot` class instance representing the vehicle to manoeuvre. 
    :param params: Set of hyperparameter values to use in the PID-controller,
        should be three values in order: [$\tau_{p}$, $\tau_{d}$, $\tau_{i}$].
    :param n: Number of time-steps to simulate.
    :param speed: Velocity (m/s) at which to drive the vehicle.
    :returns: Set of x- and y-coordinates of the simulated trajectory, plus
        an error score corresponding to the average difference in trajectories.
    """

    # Make sure all parameters were provided
    assert len(params) == 3
    tau_p, tau_d, tau_i = params
    # The list of $x$- and $y$-values for the simulated trajectory
    x_trajectory = []
    y_trajectory = []
    # The list of all previous cross-track errors (used in integral term)
    cte_values = []
    # Set the constant unit time-step value (used in derivative term)
    delta_t = 1.0
    # Initialise the "previous" and current cross-track error values
    cte_prev = robot.y
    cte_curr = 0.0
    # Store the running total square error of CTE (used to compute average)
    err = 0.0
    # Simulate the robot movement across `n` time-steps
    for i in range(2 * n):
        # Get the current cross-track error relative to reference trajectory
        cte_curr = robot.y
        cte_values.append(cte_curr)
        ### Compute the steering angle w.r.t. PID controller
        # First, caclulate the derivative term
        # NOTE: `cte_dot` is derivative of CTE w.r.t. constant unit time-step
        cte_dot = (cte_curr - cte_prev) / delta_t
        # Then, calculate the integral term 
        cte_int = np.sum(cte_values)
        # Form the expression of the complete PID controller
        # w.r.t. the proportional, integral, and derivative gain values
        steer = -tau_p * cte_curr - tau_d * cte_dot - tau_i * cte_int
        # Set the "previous" CTE value to the current error before manoeuvre
        cte_prev = cte_curr
        # Execute the steering command computed with the PID-controller
        robot.move(steer, speed)
        # Append the updated robot position coordinates to the trajectory lists
        _x, _y = robot.x, robot.y
        x_trajectory.append(_x)
        y_trajectory.append(_y)
        # Add current error to running total
        if i >= n:
            err += cte_curr ** 2
    return x_trajectory, y_trajectory, err / n


# Make this tolerance bigger if you are timing out!
def twiddle(
        tol: float=0.2
) -> Tuple[List[float], float]: 
    """Parameter optimisation algorithm using local hill-climbing.
    
    Also known as "coordinate ascent", the Twiddle algorithm used here
    optimises a set of parameters given an objective function. The
    optimisation process continues until the error falls below the given
    tolerance value `tol`.
    
    Here, we assume a set of three hyperparameters corresponding to the
    coefficients $\tau = [\tau_{i}, \tau_{d}, \tau_{i}]$, which we seek
    to optimise according to the error value as computed by the trajectory
    tracking algorithm with PID-control. This error value, our objective
    function value, is the difference between the actual- and goal
    trajectory of the robot. The error is evaluated as the cumulative average
    of the difference in the robot's coordinates $(x, y)$ and the goal
    coordinates of the reference trajectory.
    For more information: https://martin-thoma.com/twiddle/.
    See also brief video overview: https://www.youtube.com/watch?v=2uQ2BSzDvXs.
    And an in-depth review of the Twiddle algorithm performance (DE): 
    https://www.htw-mechlab.de/index.php/numerische-optimierung-in-matlab-mit-twiddle-algorithmus/

    :param tol: Tolerance value used in determining when the hyperparameter
        has converged to a good value.
    :returns: tuple, Set of modified hyperparameters and the lowest error
        returned by the objective function.
    """

    # Don't forget to call `make_robot` before every call of `run`!
    # Set the initial values of the parameters
    # Here, we assume these correspond to $[\tau_{p}, \tau_{d}, \tau_{i}]$
    p = [0., 0., 0.]
    # Set the delta values (i.e., amount to increase / decrease each parameter)
    dp = [1., 1., 1.]
    # Create and initialise the robot in its starting state
    robot = make_robot()
    # Get the error using the initial parameter values 
    _, _, best_err = run_pid_tuned(robot, p)
    iter = 0
    while sum(dp) > tol:
        print(f"Iteration #{iter}, best error: {best_err}")
        # Sequentially update each parameter in the set
        for i in range(len(p)):
            # Compute new parameter value
            p[i] += dp[i]
            # Re-evaluate error using new parameter
            robot = make_robot()
            _, _, err = run_pid_tuned(robot, p, n=100, speed=1.0)
            if err < best_err:
                # We found a better parameter value!
                # Update the lowest error found
                best_err = err
                # Update delta value to be slightly larger
                dp[i] *= 1.1
            else:
                # There was no improvement
                # Decrease parameter value ("descent")
                # Here we subtract twice the value to revert previous addition
                p[i] -= 2 * dp[i]
                # Then, re-compute the error value
                robot = make_robot()
                _, _, err = run_pid_tuned(robot, p, n=100, speed=1.0)
                if err < best_err:
                    # We found a better parameter value!
                    # Update the lowest error found
                    best_err = err
                    # Update the delta value to be slightly larger
                    dp[i] *= 1.05
                else:
                    # There was no improvement
                    # Increase parameter value in other direction ("ascent")
                    p[i] += dp[i]
                    # Decrease the delta value (step-size) to avoid overshoot
                    dp[i] *= 0.95
        # Increment the iteration number
        iter += 1
    # Compute the error corresponding to the best parameter values found
    robot = make_robot()
    _, _, best_err = run_pid_tuned(robot, p, n=100, speed=1.0)
    # Return the best parameter values and their corresponding error score
    return p, best_err


if __name__ == '__main__':
    ### Find the best parameters and corresponding error score
    # using the Twiddle coordinate ascent optimisation algorithm
    params_twiddle, err = twiddle()
    print("Final twiddle error = {}".format(err))
    ### (1) Run the trajectory tracking task with PID-controller
    # Case 1.1 : Using the parameter values obtained from Twiddle 
    x_pid_optimal_trajectory, y_pid_optimal_trajectory, err = run_pid_tuned(
            robot=make_robot(), 
            params=params_twiddle
    )
    # Case 1.2 : Using manually configured parameter values
    # Create and initialise a new `Robot` instance at its starting state
    x_pid_trajectory, y_pid_trajectory = run_pid(
            robot=make_robot(), 
            params=[0.2, 3.0, 0.1]
    )
    ### (2) Run the trajectory tracking task with PD-controller
    # Case 2.1 : Using manually configured parameter values
    x_pd_trajectory, y_pd_trajectory = run_pd(
            robot=make_robot(), 
            params=[0.2, 3.0]
    )
    ### (2) Run the trajectory tracking task with P-controller
    # Case 2.1 : Using manually configured parameter values
    x_p_trajectory, y_p_trajectory = run_p(
            robot=make_robot(), 
            params=[0.2]
    )
    ### Plot the robot coordinates relative to the reference trajectory
    fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1, 
            figsize=(24, 20), tight_layout=True
    )
    suptxt = 'Trajectory Tracking in 2D Using Proportional-, Proportional-'
    suptxt += 'Derivative, and Proportional-Integral-Derivative Controllers'
    plt.suptitle(suptxt,
            fontsize=24
    )
    txt_sup = r'Robot simulated across 100 time-steps (with steering wheel drift of $10^{\circ}$)'
    ax1.set_title(txt_sup,
            fontsize=18
    )
    txt_pid_opt = r"PID-controller with Twiddle ($\tau_{p}="
    txt_pid_opt += f"{params_twiddle[0]:.2f}$,"
    txt_pid_opt += r"$\tau_{d}="
    txt_pid_opt += f"{params_twiddle[1]:.2f}$,"
    txt_pid_opt += r"$\tau_{i}="
    txt_pid_opt += f"{params_twiddle[2]:.2f}$)"
    # NOTE: Plotting only first 100 steps of trajectory from tuned PID-controller
    ax1.plot(x_pid_optimal_trajectory[:len(x_pid_trajectory)], 
             y_pid_optimal_trajectory[:len(x_pid_trajectory)], 
             'm', label=txt_pid_opt
    )
    """
    ax1.plot(x_pid_optimal_trajectory, y_pid_optimal_trajectory,
            color='m', label=txt_pid_opt
    )
    """
    ax1.plot(x_pid_optimal_trajectory[:len(x_pid_trajectory)], 
             np.zeros(len(x_pid_trajectory)),
             'r', label='Reference'
    )
    ax2.plot(x_pid_optimal_trajectory[:len(x_pid_trajectory)], 
             y_pid_optimal_trajectory[:len(x_pid_trajectory)], 
             'm', label=txt_pid_opt
    )
    txt_pid_non_opt = 'PID-controller '
    txt_pid_non_opt += r'($\tau_{p}=0.2$, $\tau_{d}=3.0$, $\tau_{i}=0.1$)'
    ax2.plot(x_pid_trajectory, y_pid_trajectory,
             color='y', label=txt_pid_non_opt
    )
    ax2.plot(x_pd_trajectory, y_pd_trajectory, 
             color='g', label=r'PD-controller ($\tau_{p}=0.2$, $\tau_{d}=3.0$)'
    )
    ax2.plot(x_p_trajectory, y_p_trajectory, 
             color='b', label=r'P-controller ($\tau_{p}=0.2$)'
    )
    ax2.plot(x_pd_trajectory, np.zeros(len(x_pd_trajectory)), 
             color='r', label='Reference'
    )
    plt.legend(loc='lower right', fontsize='xx-large')
    plt.show()