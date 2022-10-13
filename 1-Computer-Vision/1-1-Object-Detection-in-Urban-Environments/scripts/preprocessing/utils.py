import google.protobuf
import logging
from object_detection.builders.dataset_builder import build as build_dataset
from object_detection.inputs import train_input
from object_detection.protos import input_reader_pb2
from object_detection.utils.config_util import get_configs_from_pipeline_file
import tensorflow as tf
from typing import List, Tuple, TypeVar
import waymo_open_dataset
from waymo_open_dataset import dataset_pb2 as open_dataset


### Creating a not-so-long custom variable types for typing hints
RepeatedCompositeContainer = TypeVar(google.protobuf.pyext._message.RepeatedCompositeContainer)


def get_dataset(
        path_tf_record: str, path_label_map: str
) -> tf.data.Dataset:
    """Opens a `.tfrecord` file and creates a `tf.data.Dataset` instance.

    :param path_tf_record: str, absolute path to a `.tfrecord` file.
    :param path_label_map: str, absolute path to the `.pbtxt` file,
        this should point to a StringIntLabelMap instance.
    :returns: a `tf.data.Dataset` instance, built with the input configs.
    """

    input_config = input_reader_pb2.InputReader()
    ### Set the absolute path to the locally-stored `.pbtxt` file
    input_config.label_map_path = path_label_map
    ### Reads the TF Example or TF Sequence Example protos from the
    #   local `.tfrecord` files
    input_config.tf_record_input_reader.input_path[:] = [path_tf_record]
    ### Return the `tf.data.Dataset` instance
    dataset = build_dataset(input_config)
    return dataset


def get_module_logger(mod_name: str) -> logging.Logger:
    """Initialises a console logger instance.
    
    :param mod_name: the model name to assign to the logger.
    :returns: a logger instance.
    """
    
    ### Setting up the console logger and formatter
    logger = logging.getLogger(mod_name)
    handler = logging.StreamHandler()
    formatter = logging.Formatter('%(asctime)s %(levelname)-8s %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)
    ### Prevent messages going to root handler
    logger.propagate = False
    return logger


def get_train_input(config_path: str) -> tf.data.Dataset:
    """Get the `.tfdata.Dataset` instance from the `config_path`.

    See: https://github.com/tensorflow/models/blob/master/research/object_detection/protos/train.proto

    :param config_path: str, the absolute path to the updated config file,
        a `pipeline_pb2.TrainEvalPipelineConfig` `proto` instance.
    :returns: dataset, the `tf.data.Dataset` of batched/agumented training data,
        examples are stored in a `(features, labels)` tuple.
    """

    ### Parse the config as a dictionary of configuration objects
    configs = get_configs_from_pipeline_file(config_path)
    train_config = configs['train_config']
    ### Fetch the training job configurations:
    #   e.g., batch size, preprocessing, optimiser, training steps, loss, etc.
    train_input_config = configs['train_input_config']
    # Return the `tf.data.Dataset` that holds `(features, labels)` tuple
    dataset = train_input(
                    train_config=train_config,
                    train_input_config=train_input_config, 
                    model_config=configs['model']
    )
    return dataset


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


def int64_feature(value):
    return tf.train.Feature(int64_list=tf.train.Int64List(value=[value]))


def int64_list_feature(value):
    return tf.train.Feature(int64_list=tf.train.Int64List(value=value))


def bytes_feature(value):
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=[value]))


def bytes_list_feature(value):
    return tf.train.Feature(bytes_list=tf.train.BytesList(value=value))


def float_list_feature(value):
    return tf.train.Feature(float_list=tf.train.FloatList(value=value))