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
#matplotlib.use('agg')
matplotlib.use('wxagg')
import matplotlib.pyplot as plt
### Here we use the `numpy.matrix` class
# Note that this class is being deprecated and its use in this programme will be
# switched to an `numpy.ndarray` implmenetation soon.
# See: https://numpy.org/devdocs/reference/generated/numpy.matrix.html#numpy.matrix
import numpy as np


class Camera:
    '''The Camera sensor class with non-linear measurement matrix.

    :param f_i: the focal length along the i-axis of the image plane.
    :param f_j: the focal length along the j-axis of the image plane.
    :param c_i: the component of the principal point along the i-axis.
    :param c_j: the component of the principal point along the j-axis.
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
        
    def get_hx(self,
            x: np.matrix
    ) -> np.matrix:
        """Implements the non-linear measurement function.

        :param x: the expansion point used in the first-order Taylor series.
        :returns: hx, the non-linear measurement function,
            i.e., the expectation value $h(x)$.
        """

        # calculate nonlinear measurement expectation value h(x)   
        hx = np.zeros((2,1))
        ############
        # TODO: implement and return h(x)
        ############
        return hx
    
    def get_h(self,
            x: np.matrix
    ) -> np.matrix:
        """Implements the Jacobian matrix of the camera measurement function.

        :param x: the expansion point used in the first-order Taylor series.
        :returns: H, the Jacobian matrix of the camera measurement function,
            i.e., $\mathrm{H}_{j}$ linearised about expansion point $x$.
        """

        # calculate Jacobian H at current x from h(x)
        H = np.matrix(np.zeros((2, 6)))
        ############
        # TODO: implement and return H
        ############ 
        return H
 
 
def calc_jacobian(
        x: np.matrix
):
    """Linearises the camera measurement function and plots the results.
    
    Assumed is a non-linear camera measurement function resulting from the
    projection of the 6-D state vector into the 2-D image space.

    Therefore, we compute a $2x6$ Jacobian $\mathrm{H}_{j}$, i.e.,
    the linearised camera measurement model approximated with a
    first-order Taylor expansion about the expansion point $x$.

    :param x: the expansion point used in the first-order Taylor series.
    """
    # calculate Jacobian for x
    cam = Camera()
    H = cam.get_h(x)
    # init visualization
    fig, (ax1, ax2) = plt.subplots(1,2)
    plot_x = []
    plot_y1 = []
    plot_y2 = []
    lin_y1 = []
    lin_y2 = []
    # calculate Taylor series expansion point
    hx_orig = cam.get_hx(x)
    ax1.plot(x[0], hx_orig[0],
                        marker='x', color='green', label='expansion point x'
    )
    ax2.plot(x[0], hx_orig[1],
                        marker='x', color='green', label='expansion point x'
    )
    # calculate linear approximation at this point 
    s1 = float(H[0,0]) # slope of tangent given by Jacobian H
    s2 = float(H[1,0])
    i1 = float(hx_orig[0] - s1*x[0]) # intercept i = y - s*x
    i2 = float(hx_orig[1] - s2*x[0])
    # calculate nonlinear measurement function h
    for px in range(1,50):
        x[0] = px
        hx = cam.get_hx(x)
        plot_x.append(px)
        plot_y1.append(hx[0])
        plot_y2.append(hx[1])
        lin_y1.append(s1*px + i1)
        lin_y2.append(s2*px + i2)
        
    # plot results
    ax1.plot(plot_x, plot_y1,
                        color='blue', label='measurement function h'
    )
    ax1.plot(plot_x, lin_y1,
                        color='red', label='linear approximation H'
    )
    ax2.plot(plot_x, plot_y2,
                        color='blue', label='measurement function h'
    )
    ax2.plot(plot_x, lin_y2,
                        color='red', label='linear approximation H'
    )
    # maximize window     
    mng = plt.get_current_fig_manager()
    mng.frame.Maximize(True)
    # legend
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
    plt.show()


if __name__ == '__main__':
    ### Init the camera measurement model and compute its linear approximation
    # Here we define the expansion point of the Taylor series
    x = np.matrix([[10],
                   [1],
                   [-1],
                   [0],
                   [0],
                   [0]]
    )
    # Run the linearisation programme
    calc_jacobian(x)

