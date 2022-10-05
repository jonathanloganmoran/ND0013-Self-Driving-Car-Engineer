import argparse
import io
import os
import subprocess

import ray
import tensorflow.compat.v1 as tf
from PIL import Image
from psutil import cpu_count
from waymo_open_dataset import dataset_pb2 as open_dataset

from utils import get_module_logger, parse_frame, int64_feature, int64_list_feature, \
    bytes_list_feature, bytes_feature, float_list_feature


def create_tf_example(filename, encoded_jpeg, annotations, resize=True):
    """
    This function create a tf.train.Example from the Waymo frame.

    args:
        - filename [str]: name of the image
        - encoded_jpeg [bytes]: jpeg encoded image
        - annotations [protobuf object]: bboxes and classes

    returns:
        - tf_example [tf.Train.Example]: tf example in the objection detection api format.
    """
    if not resize:
        encoded_jpg_io = io.BytesIO(encoded_jpeg)
        image = Image.open(encoded_jpg_io)
        width, height = image.size
        width_factor, height_factor = image.size
    else:
        image_tensor = tf.io.decode_jpeg(encoded_jpeg)
        height_factor, width_factor, _ = image_tensor.shape
        image_res = tf.cast(tf.image.resize(image_tensor, (640, 640)), tf.uint8)
        encoded_jpeg = tf.io.encode_jpeg(image_res).numpy()
        width, height = 640, 640

    mapping = {1: 'vehicle', 2: 'pedestrian', 4: 'cyclist'}
    image_format = b'jpg'
    xmins = []
    xmaxs = []
    ymins = []
    ymaxs = []
    classes_text = []
    classes = []
    filename = filename.encode('utf8')

    for ann in annotations:
        xmin, ymin = ann.box.center_x - 0.5 * ann.box.length, ann.box.center_y - 0.5 * ann.box.width
        xmax, ymax = ann.box.center_x + 0.5 * ann.box.length, ann.box.center_y + 0.5 * ann.box.width
        xmins.append(xmin / width_factor)
        xmaxs.append(xmax / width_factor)
        ymins.append(ymin / height_factor)
        ymaxs.append(ymax / height_factor)
        classes.append(ann.type)
        classes_text.append(mapping[ann.type].encode('utf8'))

    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': int64_feature(height),
        'image/width': int64_feature(width),
        'image/filename': bytes_feature(filename),
        'image/source_id': bytes_feature(filename),
        'image/encoded': bytes_feature(encoded_jpeg),
        'image/format': bytes_feature(image_format),
        'image/object/bbox/xmin': float_list_feature(xmins),
        'image/object/bbox/xmax': float_list_feature(xmaxs),
        'image/object/bbox/ymin': float_list_feature(ymins),
        'image/object/bbox/ymax': float_list_feature(ymaxs),
        'image/object/class/text': bytes_list_feature(classes_text),
        'image/object/class/label': int64_list_feature(classes),
    }))
    return tf_example


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


def process_tfr(path, data_dir):
    """
    process a Waymo tf record into a tf api tf record

    args:
        - path [str]: path to the Waymo tf record file
        - data_dir [str]: path to the destination directory
    """
    # create processed data dir
    dest = os.path.join(data_dir, 'processed')
    os.makedirs(dest, exist_ok=True)
    file_name = os.path.basename(path)

    logger.info(f'Processing {path}')
    writer = tf.python_io.TFRecordWriter(f'{dest}/{file_name}')
    dataset = tf.data.TFRecordDataset(path, compression_type='')
    for idx, data in enumerate(dataset):
        # we are only saving every 10 frames to reduce the number of similar
        # images. Remove this line if you have enough space to work with full
        # temporal resolution data.
        if idx % 10 == 0:
            frame = open_dataset.Frame()
            frame.ParseFromString(bytearray(data.numpy()))
            encoded_jpeg, annotations = parse_frame(frame)
            filename = file_name.replace('.tfrecord', f'_{idx}.tfrecord')
            tf_example = create_tf_example(filename, encoded_jpeg, annotations)
            writer.write(tf_example.SerializeToString())
    writer.close()


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
