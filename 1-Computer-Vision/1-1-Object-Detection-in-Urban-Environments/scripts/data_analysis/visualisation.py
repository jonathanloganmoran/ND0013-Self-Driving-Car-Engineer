from collections import Counter, namedtuple
import google.protobuf
from matplotlib import colors
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from object_detection.utils import dataset_util, label_map_util
from object_detection.builders.dataset_builder import build as build_dataset
from object_detection.utils.config_util import get_configs_from_pipeline_file
import os
import pandas as pd
import tensorflow as tf
from typing import List, Tuple, TypeVar
import waymo_open_dataset
from waymo_open_dataset import dataset_pb2 as open_dataset


### Creating some not-so-long custom variable types for typing hints
RepeatedCompositeContainer = TypeVar(
        google.protobuf.pyext._message.RepeatedCompositeContainer
)
BoxLabel = TypeVar(waymo_open_dataset.label_pb2.Label.Box)
### Creating tuple format to store normalised bounding box coordinates
BboxTuple = namedtuple("BboxTuple", ['ymin', 'ymax', 'xmin', 'xmax'])


def parse_frame(
    frame: waymo_open_dataset.dataset_pb2.Frame, 
    camera_name='FRONT'
) -> Tuple[bytes, RepeatedCompositeContainer]:
    """Returns the image and associated bounding box annotations from the frame.
    :param frame: a frame from the Waymo Open Dataset, contains images and annotations.
    :camera_name: str (optional), the camera whose image and annotations to obtain.
    :returns: tuple, the encoded image and associated bounding box data.
    """

    images = frame.images
    for im in images:
        if open_dataset.CameraName.Name.Name(im.name) != camera_name:
            continue
        else:
            # Choosing only the image from the requested camera
            encoded_jpeg = im.image
    labels = frame.camera_labels
    for lab in labels:
        if open_dataset.CameraName.Name.Name(lab.name) != camera_name:
            continue
        else:
            # Choosing only the annotations from the requested camera
            annotations = lab.labels
    return encoded_jpeg, annotations


def get_frame(
    file_path_segment: str, camera_name: str, frame_nr: int
) -> waymo_open_dataset.dataset_pb2.Frame:
    """Returns the specific `Frame` from the `.tfrecord` at the given path.

    Note that you must specify the frame number you want to extract from the
    `Frame` object.
    
    :param file_path_segment: the file path to the `.tfrecord` file.
    :param camera_name: str (optional), the name of the camera to retrieve
        the image and annotations from.
    :param frame_nr: the frame number (i.e., `image_id`) to retrieve.
    :returns: the frame corresponding to the `image_id`.
    """
    segment = tf.data.TFRecordDataset(file_path_segment, compression_type='')
    for i, data in enumerate(segment):
        if i == frame_nr:
            frame = open_dataset.Frame()
            frame.ParseFromString(bytearray(data.numpy()))
    return frame


def show_camera_image(
    camera_image, camera_labels, layout, cmap=None
):
    """Show a camera image and the given camera labels."""

    ax = plt.subplot(*layout)
    # Draw the camera labels.
    for camera_label in camera_labels:
        # Ignore camera labels that do not correspond to this camera.
        if camera_label.name != camera_image.name:
            continue
        # Iterate over the individual labels.
        for label in camera_label.labels:
            # Draw the object bounding box.
            ax.add_patch(patches.Rectangle(
                xy=(label.box.center_x - 0.5 * label.box.length,
                label.box.center_y - 0.5 * label.box.width),
                width=label.box.length,
                height=label.box.width,
                linewidth=1,
                edgecolor='red',
                facecolor='none')
            )
    # Show the camera image.
    plt.imshow(tf.image.decode_jpeg(camera_image.image), cmap=cmap)
    plt.title(open_dataset.CameraName.Name.Name(camera_image.name))
    plt.grid(False)
    plt.axis('off')



def display_instances(
    batch: tf.data.Dataset, class_colourmap, num_frames=10,
    camera_label='FRONT', **params
):
    """Renders a batch of frames and their bounding boxes.

    This function takes a batch from the dataset and display the image with 
    the associated bounding boxes in an `ImageGrid`.

    :param batch: the tf.data.BatchDataset instance of Waymo OD frames.
    :param class_colourmap: the class-colour mapping for bbox annotations.
    :param num_frames: number of frames to display in the plot.
    :param camera_label: the label of the camera to return images captured from.
    :param **params: the dict of keyword arguments to pass into Matplotlib call.
    """

    # Create the Matplotlib ImageGrid figure
    fig = plt.figure(figsize=figsize)
    grid = ImageGrid(fig, 111, nrows_ncols=(nrows, ncols), axes_pad=axes_pad)
    grid.axes_all[2].set_title(label=label, fontname=fontname)
    frame_nr = 0
    for i, data in enumerate(batch):
        if frame_nr == num_frames:
            break
        # Read in the frame from the batch
        frame = open_dataset.Frame()
        frame.ParseFromString(bytearray(data.numpy()))
        frame_nr = i+1
        # Parse the frame and obtain the images and labels
        image, labels = parse_frame(frame, camera_label)
        # Plot the image and its bounding box labels
        grid[i].imshow(tf.io.decode_image(image))
        grid[i].axis('off')
        # Iterate over the individual labels.
        for lbl in labels:
            # Draw the object bounding box.
            grid[i].add_patch(patches.Rectangle(
                xy=(lbl.box.center_x - 0.5 * lbl.box.length,
                lbl.box.center_y - 0.5 * lbl.box.width),
                width=lbl.box.length,
                height=lbl.box.width,
                linewidth=1,
                edgecolor=class_colourmap[
                    label_pb2.Label.Type.Name(lbl.type)
                ],
                facecolor='none')
            )


def make_gif(
    file_path: str, pipeline_config_path: str, output_path: str='./'
):
    """Converts frames from a Waymo `.tfrecord` file into a GIF.
    
    :param file_path: Path to the `.tfrecord` file to convert.
    :param pipeline_config_path: Path to the `pipeline.config` path of the model.
    :param output_path: Path to the destination to save the GIF file.
    """
    configs = get_configs_from_pipeline_file(config_path)
    eval_config = configs['eval_config']
    eval_input_config = configs['eval_input_config']
    eval_input_config.tf_record_input_reader.input_path[:] = [path]
    dataset = build_dataset(eval_input_config)
    images = []
    for idx, batch in enumerate(dataset):
        image_tensor = batch['image']
        image_np = image_tensor.numpy().astype(np.uint8)
        images.append(image_np)
    f = plt.figure()
    f.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=None, hspace=None)
    ax = plt.subplot(111)
    ax.axis('off')
    im_obj = ax.imshow(images[0])
    
    def animate(idx):
        image = images[idx]
        im_obj.set_data(image)
    anim = animation.FuncAnimation(f, animate, frames=50)
    anim.save(output_path, fps=5, dpi=300)

    
### To use `display_instances`, modify the following parameters: ###
# Function parameters
# num_frames = 10
# camera_label = 'FRONT'
# Bounding box colours
# class_colourmap = {
#     'TYPE_VEHICLE':    'red',
#     'TYPE_PEDESTRIAN': 'blue',
#     'TYPE_SIGN':       'yellow',
#     'TYPE_CYCLIST':    'green'
# }
# ImageGrid parameters
# figsize = (24,24)
# ncols = 5
# nrows = num_frames // ncols
# axes_pad = 0.2
# fontname = 'Times New Roman'
# label = "'FRONT' camera images in batch"
# grid_params = {'figsize': figsize, 'ncols': ncols, 'nrows': nrows, 'axes_pad': axes_pad}