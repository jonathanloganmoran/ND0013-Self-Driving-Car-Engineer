# ------------------------------------------------------------------------------
# Project "Multi-Target Tracking with Extended Kalman Filters and Sensor Fusion"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Helper functions used to evaluate and plot the results
#                        of the tracking algorithm.
#
# You should have received a copy of the Udacity license with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
#
# NOTE: The current version of this programme relies on Numpy to perform data 
#       manipulation, however, a platform-specific implementation, e.g.,
#       TensorFlow `tf.Tensor` data ops, is recommended.
# ------------------------------------------------------------------------------

import cv2
import easydict
import google.protobuf
import matplotlib
### Change Matplotlib backend for compatibility
# Using 'wxagg' backend so that figure maximizing works on Mac as well
# matplotlib.use('wxagg')
# Using 'agg' backend so that plotting works on Ubuntu 16.04.6 LTS
# Note that 'agg' is a non-GUI backend, so only figure saving will work
matplotlib.use('agg')
# matplotlib.use('wxagg')
from matplotlib import colors
import matplotlib.patches as patches
from matplotlib.path import Path
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.transforms import Affine2D
import numpy as np
import os
import sys
from typing import List, Tuple, TypeVar

### Creating some not-so-long custom variable types for typing hints
try:
    # The list of laser / camera detections type
    RepeatedCompositeContainer = TypeVar(
        google.protobuf.pyext._message.RepeatedCompositeContainer
    )
except (ModuleNotFoundError, AttributeError):
    # Out-of-date or missing `protobuf` library
    # Defaulting to generic nested list type
    RepeatedCompositeContainer = TypeVar(
        List[list]
    )
# The bounding box label type
BoxLabel = TypeVar(label_pb2.Label.Box)

### Add project directory to PYTHONPATH to enable relative imports
# Alternatively, use the `pip install ..` script with setuptools
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(
    os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__)))
)
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

### Simple Waymo Open Dataset Reader library
from tools.waymo_reader.simple_waymo_open_dataset_reader import label_pb2

### Tracking package imports
# Imports for typing hints
from student.measurements import Measurement, Sensor
from student.trackmanagement import Track, Trackmanagement


def plot_tracks(
        fig: Matplotlib.figure.Figure,
        ax: matplotlib.axis.Axis,
        ax2: matplotlib.axis.Axis,
        track_list: List[Track],
        meas_list: List[Measurement],
        lidar_labels: RepeatedCompositeContainer,
        lidar_labels_valid: np.ndarray,
        image: np.ndarray,
        camera: Sensor,
        configs_det: easydict.EasyDict,
        state: bool=None
) -> Tuple[Matplotlib.figure.Figure, matplotlib.axis.Axis, Matplotlib.axis.Axis]:
    """Plots the tracks, measurements and ground-truth labels in BEV image.

    :param fig: the `Matplotlib.figure.Figure` subplot instance to update with
        the given tracks and measurements; is returned after modification.
    :param ax: the `Matplotlib.axis.Axis` instance of the first subplot.
    :param ax2: the `Matplotlib.axis.Axis` instance of the second subplot.
    :param track_list: the list of Track objects corresponding to the object
        tracks, each with a unique `id`, `state`, `score`, dimensions, etc.
    :param meas_list: the list of Measurement objects corresponding to the
        object tracks in `track_list`.
    :param lidar_labels: the ground-truth bounding box annotations from the
        `frame.laser_labels` attribute as a protocol buffer message type. 
    :param lidar_labels_valid: a list of boolean flags, each map to a given
        track, here a value `1` indicates that the track corresponds to an
        object of the 'TYPE_VEHICLE' class whose bbox location was inside the
        BEV image frame defined by `configs_det.lim_{}` for `x`, `y`, and `z`. 
    :param image: the image captured by the FRONT camera as a Numpy `ndarray`.
    :param camera: the Sensor instance for the camera sensor used to generate
        the object tracks.
    :param configs_det: the `EasyDict` instance containing the BEV image
        dimensions, used to limit the plot axes / clip unwanted detections.
    :param state: optional, will only plot tracks with the given `track.state`
        value, can be one of `['initialized', 'tentative', 'confirmed']`. 
    :returns: tuple, the Matplotlib figure and axes modified with the plotted
        results.
    """
    
    ### Initialise the Matplotlib figure
    # Clear the subplots
    ax.cla()
    ax2.cla()
    # Plot the image onto the second subplot
    ax2.imshow(image)
    ### Loop through all tracks in list
    for track in track_list:
        ### Plot each track, measurement and ground truth in BEV
        # Optionally, restrict the plotted tracks to those with a given state
        # If no state was provided, plot all input tracks
        if track.state == state or state == None:
            # Choose the colour of the track to plot according to its state
            if track.state == 'confirmed':
                col = 'green'
            elif track.state == 'tentative':
                col = 'orange'
            else:
                # e.g., 'initialized'
                col = 'red'
            # Get the current state variables associated with this track    
            w = track.width
            h = track.height
            l = track.length
            x = track.x[0]
            y = track.x[1]
            z = track.x[2]
            yaw = track.yaw 
            # Create a Rectange patch for each bounding box
            # after transforming the coordinates to top BEV
            t = Affine2D().rotate_around(*(0, 0), -yaw)
            t += Affine2D().translate(-y, x) + ax.transData
            point_of_rotation = np.array([w / 2, l / 2])        
            rec = plt.Rectangle(
                        -point_of_rotation,
                        width=w,
                        height=l,
                        color=col,
                        alpha=0.2,
                        transform=t
            ax.add_patch(rec)
            # Plot the track `id` for debugging
            ax.text(
                x=float(-track.x[1]),
                y=float(track.x[0]+1),
                s=str(track.id)
            )
            # Plot the track position depending on its state
            if track.state == 'initialized':
                ax.scatter(
                    x=float(-track.x[1]),
                    y=float(track.x[0]),
                    color=col,
                    s=80,
                    marker='x',
                    label='initialized track'
                )
            elif track.state == 'tentative':
                ax.scatter(
                    x=float(-track.x[1]),
                    y=float(track.x[0]),
                    color=col,
                    s=80,
                    marker='x',
                    label='tentative track'
                )
            elif track.state == 'confirmed':
                ax.scatter(
                    x=float(-track.x[1]),
                    y=float(track.x[0]),
                    color=col,
                    s=80,
                    marker='x',
                    label='confirmed track'
            )
            ### Project the tracks into image space
            # Transform the coordinates from vehicle to sensor frame
            # Here we construct a homogeneous coordinate system
            pos_veh = np.ones((4, 1))
            pos_veh[0:3] = track.x[0:3]
            pos_sens = camera.veh_to_sens @ pos_veh
            # Obtain the transformed coordinates in image space
            x, y, z = pos_sens
            ### Obtain the coordinates of the bounding box
            # Get the coordinates of the bounding box corners
            x_corners = [-l/2, l/2, l/2, l/2, l/2, -l/2, -l/2, -l/2]
            y_corners = [-w/2, -w/2, -w/2, w/2, w/2, w/2, w/2, -w/2]
            z_corners = [-h/2, -h/2, h/2, h/2, -h/2, -h/2, h/2, h/2]
            # Create a Numpy array of all bounding box corners
            corners_3D = np.array([x_corners, y_corners, z_corners])
            ### Transform the coordinates from image to world coordinates
            # Compute the rotation matrix around the z-axis
            R = np.matrix([
                    [np.cos(yaw), np.sin(yaw), 0.],
                    [-np.sin(yaw), np.cos(yaw), 0.],
                    [0., 0., 1.]
            ])
            # Transform the coordinates with a rotation around z-axis
            corners_3D = R * corners_3D
            # Translate the corner coordinates
            corners_3D += np.array([x, y, z]).reshape((3, 1))
            # print ( 'corners_3d', corners_3D)
            # TODO: Remove bounding box that includes negative `x` value
            # where a projection would make no sense
            if np.any(corners_3D[0,:] <= 0):
                continue
            ### Project coordinates to image space
            corners_2D = np.zeros((2, 8))
            for k in range(8):
                corners_2D[0, k] = camera.c_i - camera.f_i * corners_3D[1, k] / corners_3D[0, k]
                corners_2D[1, k] = camera.c_j - camera.f_j * corners_3D[2, k] / corners_3D[0, k]
            # print ( 'corners_2d', corners_2D)
            # Create the edges of bounding box in vertex index from above
            # e.g. index 0 stands for [-l / 2, -w / 2, -h / 2]
            draw_line_indices = [0, 1, 2, 3, 4, 5, 6, 7, 0, 5, 4, 1, 2, 7, 6, 3]
            paths_2D = np.transpose(corners_2D[:, draw_line_indices])
            # print ( 'paths_2D', paths_2D)
            codes = [Path.LINETO] * paths_2D.shape[0]
            codes[0] = Path.MOVETO
            path = Path(paths_2D, codes)
            ### Create a PathPatch the resulting bounding box
            ax2.add_patch(
                patches.PathPatch(
                    path,
                    fill=False,
                    color=col,
                    linewidth=3
                )
            )
    ### Plot the track labels
    for label, valid in zip(lidar_labels, lidar_labels_valid):
        if valid:        
            ax.scatter(
                x=-1 * label.box.center_y,
                y=label.box.center_x,
                color='gray',
                s=80,
                marker='+',
                label='ground truth'
            )
    ### Plot the measurements
    for meas in meas_list:
        ax.scatter(
            x=-1 * meas.z[1],
            y=meas.z[0],
            color='blue',
            marker='.',
            label='measurement'
        )
    ### Configure the Matplotlib figure instance
    # Maximise the figure window if using `wxagg` backend
    if matplotlib.rcParams['backend'] == 'wxagg':
        mng = plt.get_current_fig_manager()
        mng.frame.Maximize(True)
    # Set the x- and y-axis labels
    ax.set_xlabel('y [m]')
    ax.set_ylabel('x [m]')
    # Set the aspect ratio of the axes scaling
    ax.set_aspect('equal')      # Using same scaling for x- and y-axis
    # Set the x- and y-axis limits for corresponding vehicle coordinate frame
    # So that positive x points forward and positive y points left
    ax.set_ylim(configs_det.lim_x[0], configs_det.lim_x[1])
    ax.set_xlim(-configs_det.lim_y[1], -configs_det.lim_y[0])
    # Correct the x-axis ticks so that positive values are to the left
    ticks_x = ticker.FuncFormatter(
        lambda x, pos: '{0:g}'.format(-x) if x != 0 else '{0:g}'.format(x)
    )
    ax.xaxis.set_major_formatter(ticks_x)
    # Remove any repeated labels from the plot
    handles, labels = ax.get_legend_handles_labels()
    handle_list, label_list = [], []
    for handle, label in zip(handles, labels):
        if label not in label_list:
            handle_list.append(handle)
            label_list.append(label)
    # Initialise the legend
    ax.legend(handle_list, label_list, loc='center left',
            shadow=True, fontsize='x-large', bbox_to_anchor=(0.8, 0.5)
    )
    # Set the refresh time (used to animate the figure)
    plt.pause(0.01)
    # Return the modified figure and subplot axes
    return fig, ax, ax2


def plot_rmse(
        manager: Trackmanager,
        all_labels: List[list],
        configs_det: easydict.EasyDict=None
):
    """Calculates the RMSE score for the given tracks with ground-truth labels.

    The list of ground-truth detections, i.e., `all_labels` is assumed to be a
    list of lists, where each element has the form:
            `[frame.laser_labels, valid_label_flags]`
    where `valid_label_flags` is an array with elements `1` or `0` such that
    `1` indicates a valid object, i.e., one that is of the 'TYPE_VEHICLE'
    object class and has a corresponding bounding box label inside the default
    BEV image range (see below).

    If the `configs_det` instance is not provided, the provided tracks are
    assumed to be within the default BEV image range, i.e., between
            0 < x < 50        and        -25 < y < 25.
    Note that when using the default implementation, `all_labels` should have
    been already sanity-checked using `objdet_tools.validate_object_labels()`.

    :param manager: the Trackmanager instance containing the 'confirmed' tracks
        to evaluate.
    :param all_labels: the list of ground-truth detections, each element has the
        form `[frame.laser_labels, valid_label_flags]` where 
    """

    ### Initialise a Matplotlib instance
    fig, ax = plt.subplots()
    plot_empty = True
    ### Loop over all tracks
    for track_id in range(manager.last_id+1):
        # Initialise the variables needed to compute RMSE
        rmse_sum = 0
        cnt = 0
        rmse = []
        time = []
        ### Loop over the tracks from each time-step
        for i, result_dict in enumerate(manager.result_list):
            # Get the ground-truth bounding box labels
            label_list = all_labels[i]
            # Skip track if no estimate exists for this time-step
            if track_id not in result_dict:
                continue
            # Get the track info associated with the track id
            track = result_dict[track_id]
            if track.state != 'confirmed':
                # Skipping any tracks with score below the 'confirmed' threshold
                continue
            ### Find the closest label and calculate error at this time-step
            min_error = np.inf
            for label, valid in zip(label_list[0], label_list[1]):
                error = 0
                if valid: 
                    ### Check if the label lies inside specified BEV range
                    # Note this is redundant if using the default BEV range
                    # Therefore, `configs_det` should only be provided as an
                    # argument when different BEV dimensions are used
                    is_inside_bev_dims = True 
                    if configs_det:
                        if not (
                            label.box.center_x > configs_det.lim_x[0]
                            and label.box.center_x < configs_det.lim_x[1]
                            and label.box.center_y > configs_det.lim_y[0]
                            and label.box.center_y < configs_det.lim_y[1]
                        ):
                            is_inside_bev_dims = False
                    elif is_inside_bev_dims:
                        # Compute error for the track position estimate
                        error += (label.box.center_x - float(track.x[0]))**2
                        error += (label.box.center_y - float(track.x[1]))**2
                        error += (label.box.center_z - float(track.x[2]))**2
                        if error < min_error:
                            min_error = error
                    else:
                        raise 'Warning: label not inside BEV detection window'
            # Check that the track error was calculated
            if min_error < np.inf:
                # Compute the RMSE score
                error = np.sqrt(min_error)
                time.append(track.t)
                rmse.append(error)
                rmse_sum += error
                cnt += 1
        ### Calculate the overall RMS for each track
        if cnt:
            plot_empty = False
            # Compute the average RMSE over all time-steps
            rmse_sum /= cnt
            # Plot the resulting average RMSE score
            ax.plot(
                time,
                rmse,
                marker='x',
                label=f'RMSE track {track_id}\n (mean: {rmse_sum:.2f})'
            )
    ### Configure the Matplotlib figure instance
    # Maximise the figure window if using `wxagg` backend
    if matplotlib.rcParams['backend'] == 'wxagg':
        mng = plt.get_current_fig_manager()
        mng.frame.Maximize(True)
    # Set the y-axis limits
    ax.set_ylim(0, 1)
    if plot_empty: 
        print('No confirmed tracks found to plot RMSE!')
    else:
        # Configure the figure legend
        plt.legend(loc='center left',
            shadow=True, fontsize='x-large', bbox_to_anchor=(0.9, 0.5)
        )
        # Set the x- and y-axis labels
        plt.xlabel('time [s]')
        plt.ylabel('RMSE [m]')
        ### Show the resulting plot
        if matplotlib.rcParams['backend'] != 'agg':
            # If using a GUI backend, display the plot in a new window
            plt.show()
        else:
            # Using a non-GUI backend, save the figure to an image file
            plt.savefig('2022-11-18-Figure-1-Evaluation-RMSE.png')


def make_movie(
        path: str
):
    """Creates an animation of the tracking results for the given files.

    :param path: the absolute path to the subfolder containing the images
        to render in an animation file ('.avi' extension).
    """
    
    # Obtain the images from the folder pointed to by `path`
    images = [img for img in sorted(os.listdir(path)) if img.endswith(".png")]
    frame = cv2.imread(os.path.join(path, images[0]))
    height, width, layers = frame.shape
    # Create a 10 fps video and save it in the folder pointed to by `path`
    video = cv2.VideoWriter(
        filename=os.path.join(path, 'my_tracking_results.avi'),
        fourcc=0,
        fps=10,
        frameSize=(width,height)
    )
    for image in images:
        # Write each image to the file
        fname = os.path.join(path, image)
        video.write(cv2.imread(fname))
        # Remove the frame from the folder
        os.remove(fname)
    cv2.destroyAllWindows()
    video.release()