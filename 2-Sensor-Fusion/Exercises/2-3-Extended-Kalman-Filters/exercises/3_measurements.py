# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Define the `Camera` sensor class and its core
#                        functions, i.e., the non-linear measurement model
#                        and the intrinsic camera parameters.                        
#
# You should have received a copy of the Udacity license with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
#
# NOTE: The current version of this programme relies on Numpy to perform data 
#       manipulation, however, a platform-specific implementation, e.g.,
#       TensorFlow `tf.Tensor` data ops, is recommended.
# ------------------------------------------------------------------------------

import matplotlib
### Change Matplotlib backend for compatibility
# Using 'wxagg' backend so that figure maximizing works on Mac as well
# matplotlib.use('wxagg')
# Using 'agg' backend so that plotting works on Ubuntu 16.04.6 LTS
# Note that 'agg' is a non-GUI backend, so only figure saving will work
# matplotlib.use('agg')
# matplotlib.use('wxagg')
import matplotlib.pyplot as plt
import numpy as np


class Camera:
    '''The Camera sensor class with non-linear measurement matrix.

    :param f_i: the focal length along the i-axis of the image plane.
    :param f_j: the focal length along the j-axis of the image plane.
    :param c_i: the component of the principal point along the i-axis.
    :param c_j: the component of the principal point along the j-axis.
    :param hx: the non-linear camera measurement function.
    :param H: the linearised camera measurement matrix as a Jacobian.
    '''

    def __init__(self):
        """Initialises the Camera object with given intrinsics."""

        # The focal length along the i-axis
        self.f_i = 2095.5
        # The focal length along the j-axis
        self.f_j = 2095.5
        # The principal point along the i-axis
        self.c_i = 944.9
        # The principal point along the j-axis
        self.c_j = 640.2
        # The non-linear camera measurement function
        self.hx = None
        # The linearised camera measurement matrix as a Jacobian
        self.H = None
        
    def get_hx(self,
            x: np.ndarray
    ) -> np.ndarray:
        """Implements the non-linear measurement function.

        :param x: the expansion point used in the first-order Taylor series.
        :returns: hx, the non-linear measurement function,
            i.e., the expectation value $h(x)$.
        """

        ### Calculate the non-linear measurement expectation value $h(x)$
        # Obtain the position coordinates from the state vector `x`
        p_x, p_y, p_z = x[0:3]
        # Define the non-zero entries of the expectation value
        h11 = self.c_i - self.f_i * p_y / p_x
        h21 = self.c_j - self.f_j * p_z / p_x
        # Set the non-zero entries with the computed values
        hx = np.array([[h11],
                       [h21]])
        # Update the class attribute and return the expectation value vector
        self.hx = hx
        return hx
    
    def get_h(self,
            x: np.ndarray
    ) -> np.ndarray:
        """Implements the Jacobian matrix of the camera measurement function.

        Assumed is the zero contribution of the velocity term of the state
        vector in the measurement function. Here the first-order Taylor expansion
        is commputed to linearise the camera measurement function at the expansion
        point $x$.

        :param x: the expansion point used in the first-order Taylor series.
        :returns: H, the Jacobian matrix of the camera measurement function,
            i.e., $\mathrm{H}_{j}$ linearised about expansion point $x$.
        """

        ### Calculate the Jacobian `H` at current `x` of $h(x)$
        # Obtain the position coordinates from the state vector `x`
        p_x, p_y, p_z = x[0:3]
        # Define the non-zero entries of the Jacobian `H`
        # Variable naming convention is matrix [row, col] indexing starting at 1
        h11 = self.f_i * p_y / p_x**2
        h12 = -1 * self.f_i / p_x
        h21 = self.f_j * p_z / p_x**2
        h23 = -1 * self.f_j / p_x
        # Set the non-zero entries with the computed values
        H = np.array([[h11, h12, 0., 0., 0., 0.],
                      [h21, 0., h23, 0., 0., 0.]])
        # Update the class attribute and return the Jacobian `H`
        self.H = H
        return H
 
def calc_jacobian(
        x: np.ndarray
):
    """Linearises the camera measurement function and plots the results.
    
    Assumed is a non-linear camera measurement function resulting from the
    projection of the 6-D state vector into the 2-D image space.

    Therefore, we compute a $2x6$ Jacobian $\mathrm{H}_{j}$, i.e.,
    the linearised camera measurement model approximated with a
    first-order Taylor expansion about the expansion point $x$.

    :param x: the expansion point used in the first-order Taylor series.
    """

    ### Calculate the Jacobian for state vector `x`
    cam = Camera()
    H = cam.get_h(x)
    ### Visualise the results
    # Define the Matplotlib figure instance and data arrays
    fig, (ax1, ax2) = plt.subplots(1, 2)
    plot_x = []
    plot_y1 = []
    plot_y2 = []
    lin_y1 = []
    lin_y2 = []
    # Calculate the expansion point `x` for the first-order Taylor series
    hx_orig = cam.get_hx(x)
    # Plot the expansion point coordinates
    ax1.plot(x[0], hx_orig[0],
                        marker='x', color='green', label='expansion point x'
    )
    ax2.plot(x[0], hx_orig[1],
                        marker='x', color='green', label='expansion point x'
    )
    ### Calculate the linear approximation at this point
    # Define the slope of the tangent given by the Jacobian `H`
    s1 = float(H[0, 0])
    s2 = float(H[1, 0])
    # Define the intercept of the slope to be $i = y - s * x$
    i1 = float(hx_orig[0] - s1 * x[0])
    i2 = float(hx_orig[1] - s2 * x[0])
    # Calculate the non-linear measurement function $h(x)$ for each point in `x`
    for px in range(1, 50):
        # Get the expansion point for position `px`
        x[0] = px
        hx = cam.get_hx(x)
        # Append the results for plotting
        plot_x.append(px)
        plot_y1.append(hx[0])
        plot_y2.append(hx[1])
        lin_y1.append(s1 * px + i1)
        lin_y2.append(s2 * px + i2)
    ### Plot the results obtained from the linearisation of $h(x)$
    plot_x, plot_y1 = np.asarray(plot_x), np.asarray(plot_y1).squeeze()
    ax1.plot(plot_x.tolist(), plot_y1.tolist(),
                        color='blue', label='measurement function h'
    )
    lin_y1 = np.asarray(lin_y1).squeeze()
    ax1.plot(plot_x.tolist(), lin_y1.tolist(),
                        color='red', label='linear approximation H'
    )
    plot_y2 = np.asarray(plot_y2).squeeze()
    ax2.plot(plot_x.tolist(), plot_y2.tolist(),
                        color='blue', label='measurement function h'
    )
    lin_y2 = np.asarray(lin_y2).squeeze()
    ax2.plot(plot_x.tolist(), lin_y2.tolist(),
                        color='red', label='linear approximation H'
    )
    # Maximise the figure window
    mng = plt.get_current_fig_manager()
    mng.frame.Maximize(True)
    # Display the plot legends and set axes labels
    ax1.legend(loc='center left',
                        shadow=True, fontsize='large', bbox_to_anchor=(0.5, 0.1)
    )
    ax1.set_xlabel('x [m]')
    ax1.set_ylabel('h(x) first component [px]')
    ax2.legend(loc='center left',
                        shadow=True, fontsize='large', bbox_to_anchor=(0.5, 0.1)
    )
    ax2.set_xlabel('x [m]')
    ax2.set_ylabel('h(x) second component [px]')
    # Show the Matplotlib figure
    plt.show()


if __name__ == '__main__':
    ### Init the camera measurement model and compute its linear approximation
    # Here we define the expansion point of the Taylor series
    x = np.array([[10],
                  [1],
                  [-1],
                  [0],
                  [0],
                  [0]]
    )
    # Run the linearisation programme
    calc_jacobian(x)

