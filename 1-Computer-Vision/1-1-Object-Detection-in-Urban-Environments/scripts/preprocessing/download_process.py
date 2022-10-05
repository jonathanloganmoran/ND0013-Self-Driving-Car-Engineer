import argparse
from collections import namedtuple
import google.protobuf
import io
from PIL import Image
from psutil import cpu_count
import os
from object_detection.utils import dataset_util, label_map_util
import ray
import subprocess
import tensorflow as tf
from typing import List, TypeVar
from utils import bytes_list_feature, bytes_feature, float_list_feature
from utils import get_module_logger
from utils import int64_feature, int64_list_feature, parse_frame
from waymo_open_dataset import dataset_pb2 as open_dataset


### Creating some not-so-long custom variable types for typing hints
RepeatedCompositeContainer = TypeVar(google.protobuf.pyext._message.RepeatedCompositeContainer)
BoxLabel = TypeVar(waymo_open_dataset.label_pb2.Label.Box)
### Creating tuple format to store normalised bounding box coordinates
BboxTuple = namedtuple("BboxTuple", ['ymin', 'ymax', 'xmin', 'xmax'])
### Fetching the Label Map file and building inverted label map dict
label_map = label_map_util.load_labelmap(PATH_TO_LABEL_MAP)
label_map_dict = label_map_util.get_label_map_dict(label_map)
label_map_dict_inverted = {cls_id:cls_label for cls_label, cls_id in label_map_dict.items()}


def _build_bounding_box(open_dataset_box: waymo_open_dataset.label_pb2.Label.Box,
                        image_width: int, image_height: int) -> BboxTuple:
    """Builds and returns coordinates of normalised bounding box.
    
    Credit: https://github.com/tensorflow/datasets/blob/master/tensorflow_datasets/object_detection/waymo_open_dataset.py
    
    :param open_dataset_box: Bounding box feature with the centre x-y
        coordinates, box height and width.
    :param image_width: Width of the Waymo Open Dataset image.
    :param image_height: Height of the Waymo Open Dataset image.
    :returns: tuple of normalised bbox coordinates.
    """
    center_y = open_dataset_box.center_y
    center_x = open_dataset_box.center_x
    length = open_dataset_box.length
    width = open_dataset_box.width
    return BboxTuple(
        ymin=max((center_y - (width / 2)) / image_height, 0.0),
        ymax=min((center_y + (width / 2)) / image_height, 1.0),
        xmin=max((center_x - (length / 2)) / image_width, 0.0),
        xmax=min((center_x + (length / 2)) / image_width, 1.0)
    )


def _get_label_from_id(label: int) -> str:
    """Returns the class label of the input class id."""
    return label_map_dict_inverted[label]


def create_tf_example(filename: str, encoded_jpeg: bytes, 
    annotations: RepeatedCompositeContainer, resize: bool=True) -> tf.train.Example:
    """Converts to TensorFlow Object Detection API format.

    Annotations assumed to be labels from one camera. 
   
    :param filename: the name and extension of the image file.
    :param encoded_jpeg: the byte-encoded jpeg image.
    :param annotations: the container object storing bboxes and classes.
    :param resize: bool (optional), resizes images to 640x640px, True by default.
    :returns: a tf.train.Example-formatted data sample.
    """

    if not resize:
        # Fetch encoded image as binary stream
        encoded_jpeg_io = io.BytesIO(encoded_jpeg)
        image = Image.open(encoded_jpeg_io)
        # Store image info
        height, width = image.size
        height_factor, width_factor = image.size
    else:
        image_tensor = tf.io.decode_image(encoded_jpeg)
        height_factor, width_factor, _ = image_tensor.shape
        image_res = tf.cast(tf.image.resize(image_tensor, size=(640, 640)), dtype=tf.uint8)
        encoded_jpeg = tf.io.encode_jpeg(image_res).numpy()
        width, height = 640, 640

    filename = filename.encode('utf8')
    image_format = b'jpeg'
    # Lists to store all bounding box features
    ymins = []
    ymaxes = []
    xmins = []
    xmaxes = []
    # Lists to store all class features
    class_labels = []
    class_ids = []
    # Iterate through each row in the annotations set
    for index, row in enumerate(annotations):
        # Get normalised bounding box coordinates
        (ymin, ymax, 
         xmin, xmax) = _build_bounding_box(row.box, width_factor, height_factor)
        ymins.append(ymin)
        ymaxes.append(ymax)
        xmins.append(xmin)
        xmaxes.append(xmax)
        # Get class info
        class_labels.append(_get_label_from_id(row.type).encode('utf8'))
        class_ids.append(row.type)
    # Create TF Features tensor
    tf_features = tf.train.Features(feature={
        # Store image features
        'image/height': int64_feature(height),
        'image/width': int64_feature(width),
        'image/filename': bytes_feature(filename),
        'image/source_id': bytes_feature(filename),
        'image/encoded': bytes_feature(encoded_jpeg),
        'image/format': bytes_feature(image_format),
        # Store object features
        'image/objects/bboxes/ymins': float_list_feature(ymins),
        'image/objects/bboxes/ymaxes': float_list_feature(ymaxes),
        'image/objects/bboxes/xmins': float_list_feature(xmins),
        'image/objects/bboxes/xmaxes': float_list_feature(xmaxes),
        # Store class features
        'image/objects/class/labels': bytes_list_feature(class_labels),
        'image/objects/class/ids': int64_list_feature(class_ids)
    })
    return tf.train.Example(features=tf_features)


def download_tfr(filename, data_dir):
    """
    download a single tf record

    args:
        - filename [str]: path to the tf record file
        - data_dir [str]: path to the destination directory

    returns:
        - local_path [str]: path where the file is saved
    """
    # create data dir
    dest = os.path.join(data_dir, 'raw')
    os.makedirs(dest, exist_ok=True)

    # download the tf record file
    cmd = ['gsutil', 'cp', filename, f'{dest}']
    logger.info(f'Downloading {filename}')
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if res.returncode != 0:
        logger.error(f'Could not download file {filename}')

    local_path = os.path.join(dest, os.path.basename(filename))
    return local_path


def process_tfr(file_path: str, dest_dir: str):
    """Process a Waymo data file into a TF-compatible type.

    Formats the Waymo Open Data `.tfrecord` type into a 
    `tf.train.Example` type for use with the TF Object Detection API.

    :param file_path: Absolute path pointing to the `.tfrecord` to convert.
    :param dest_dir: Absolute path pointing to the destination directory.
    """

    # Create subdirectory for output files
    os.makedirs(dest_dir, exist_ok=True)
    # Get the name of file to convert
    file_name = os.path.basename(file_path)
    # Get the path to directory where file is located
    src_dir = os.path.dirname(file_path)
    # Return if converted file already exists
    if os.path.exists(f'{dest_dir}/{file_name}'):
        print('Skipping {}, file already exists!'.format(file_name))
        return
    else:
        logging.info(f'Processing {file_path}')
        #writer = tf.python_io.TFRecordWriter(f'output/{file_name}')
        ### FIX: Updated in TensorFlow 2.0
        # Credit: https://github.com/datitran/raccoon_dataset/issues/90#issuecomment-647073794
        #writer = tf.compat.v1.python_io.TFRecordWriter(f'output/{file_name}')
        #writer = tf.io.TFRecordWriter(f'output/{file_name}')
        writer = tf.io.TFRecordWriter(f'{dest_dir}/{file_name}')
        dataset = tf.data.TFRecordDataset(file_path, compression_type='')
        # For each Frame in the Waymo OD `.tfrecord`
        for idx, data in enumerate(dataset):
            frame = open_dataset.Frame()
            frame.ParseFromString(bytearray(data.numpy()))
            # Fetch the image and annotations in the Frame
            encoded_jpeg, annotations = parse_frame(frame)
            filename = file_name.replace('.tfrecord', f'_{idx}.tfrecord')
            tf_example = create_tf_example(filename, encoded_jpeg, annotations, resize=True)
            # Write the serialised tf.train.Example to the TFRecordDataset
            writer.write(tf_example.SerializeToString())
        writer.close()
    return


@ray.remote
def download_and_process(filename, data_dir):
    logger = get_module_logger(__name__)
    # need to re-import the logger because of multiprocesing
    local_path = download_tfr(filename, data_dir)
    process_tfr(local_path, data_dir)
    # remove the original tf record to save space
    logger.info(f'Deleting {local_path}')
    os.remove(local_path)


if __name__ == "__main__":
    logger = get_module_logger(__name__)
    parser = argparse.ArgumentParser(description='Download and process tf files')
    parser.add_argument('--data_dir', required=True,
                        help='data directory')
    parser.add_argument('--size', required=False, default=100, type=int,
                        help='Number of files to download')
    args = parser.parse_args()
    data_dir = args.data_dir
    size = args.size

    # open the filenames file
    with open('filenames.txt', 'r') as f:
        filenames = f.read().splitlines()
    logger.info(f'Download {len(filenames[:size])} files. Be patient, this will take a long time.')

    # init ray
    ray.init(num_cpus=cpu_count())
    workers = [download_and_process.remote(fn, data_dir) for fn in filenames[:size]]
    _ = ray.get(workers)
