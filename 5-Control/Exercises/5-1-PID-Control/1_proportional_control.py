# -----------------------------------------------------------------------------
# Lesson "5.1: PID Control"
# Authors     : Sebastian Thrun.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file: Implements the Proportional Controller and simulates
#                       robot motion along a horizontal reference trajectory.
# -----------------------------------------------------------------------------

import matplotlib.pyplot as plt
import numpy as np
import random
from typing import List, Tuple


class Robot(object):
    '''The simulated Robot class.
    
    The robot moves about the 2D environment given a position (x, y) and
    heading (`orientation`) angle, i.e., direction of motion.

    The proportional controller used here follows the equation:
        $\alpha = -\tau * \mathrm{CTE}$,
    where the steering input angle $\alpha$ is computed w.r.t. the
    cross-track error $\mathrm{CTE}$ proportional to the gain factor
    $\tau$ (i.e., the "response strength"). The cross-track error is
    defined w.r.t. reference trajectory, which here is assumed to be
    a constant horizontal trajectory along the x-axis.   
    
    The proportional controller implemented here is used to direct the
    robot motion towards the horizontal reference trajectory by giving
    steering angle commands computed w.r.t. the normally-distributed
    steering drift, as well as both steering- and distance measurement
    noise.

    :param x: Position of the robot along the $x$-axis.
    :param y: Position of the robot along the $y$-axis.
    :param orientation: Heading angle of the robot (i.e., the direction).
    :param length:
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

        :param steering: Front-wheel steering angle.
        :param distance: Total distance driven by the vehicle,
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
        # Applying the steering drift parameter
        steering2 += self.steering_drift
        # Compute the manoeuvre / turn motion to execute
        turn = np.tan(steering2) * distance2 / self.length
        if abs(turn) < tolerance:
            # Compute the straight-line distance approximation of the manoeuvre
            self.x += distance2 * np.cos(self.orientation)
            self.y += distance2 * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # Evaluate the approximate kinematic bicycle motion model
            # for the desired turning manoeuvre
            radius = distance2 / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)

    def __repr__(self):
        """Overrides the default print function with vehicle state values."""
        return '[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)


def run(
        robot: Robot, 
        tau: float, 
        n: int=100, 
        speed: float=1.0
) -> Tuple[List[float], List[float]]:
    """Simulates the robot movement across `n` time-steps.
    
    The robot's steering angle $\alpha$ is computed relative to the
    proportional gain controller, which follows the equation:
        $\alpha = -\tau * \mathrm{CTE}$,
    where the cross-track error $\mathrm{CTE}$ is defined as the
    robot position along the $y$-axis, assuming a horizontal reference
    trajectory about the $x$-axis. Here, the gain factor $\tau$ is proportional
    to the cross-track error $\mathrm{CTE}$.

    :param robot: `Robot` class instance representing the vehicle to manoeuvre. 
    :param tau: Proportional gain constant.
    :param n: Number of time-steps to simulate.
    :param speed: Velocity (m/s) at which to drive the vehicle.
    """

    # The list of $x$- and $y$-values for the simulated trajectory
    x_trajectory = []
    y_trajectory = []
    # Simulate the robot movement across `n` time-steps
    for i in range(n):
        # Get the current cross-track error relative to reference trajectory
        cte = robot.y
        # Compute the steering angle w.r.t. the proportional gain controller
        steer = -tau * cte
        # Execute the steering command
        robot.move(steer, speed)
        # Append the updated robot position coordinates to the trajectory lists
        _x, _y = robot.x, robot.y
        x_trajectory.append(_x)
        y_trajectory.append(_y)
    return x_trajectory, y_trajectory


if __name__ == '__main__':
    ### Initialise the robot instance using default values
    robot = Robot()
    ### Perform a single control run to position (0, 1)
    # with heading angle `0.0`
    robot.set(0.0, 1.0, 0.0)
    ### Execute the trajectory using the proportional controller            
    x_trajectory, y_trajectory = run(robot, 0.1)
    n = len(x_trajectory)
    ### Plot the robot position and orientation relative to the reference
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))
    ax1.plot(x_trajectory, y_trajectory, 'g', label='P controller')
    ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')
    plt.show()
