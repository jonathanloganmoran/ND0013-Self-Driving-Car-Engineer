# TensorFlow Object Detection API on Custom Dataset #
# Packaged with <3 by Jonathan L. Moran (jonathan.moran107@gmail.com).

r"""Set of functions to download and process a list of `.tfrecord` GCS-hosted files.

In order to download and convert the files to `tf.data.TFRecordDataset` instances, the following must be provided:
    * `label_map_path`: the path to the `label_map.pbtxt` file;
    * `data_dir`: the path to the data directory containing a `filenames.txt` file;
       * `filenames.txt` should be a list of GCS paths;
    * `size`: the number of records from `filenames.txt` to download.

You may specify these in `configs/dataset/` or override them at runtime (see below).

To download and process the `.tfrecord` files from Google Cloud Storage into `tf.data.TFRecordDataset` instances, run:

    ```python
    python3 download_process.py
    ```

    with none/any/all of the following parameters:
        DATA_DIR:        str         Path to the `data` directory to download files to.
        LABEL_MAP_PATH:  str         Path to the dataset `label_map.pbtxt` file.
        SIZE:            str         Number of `.tfrecord` files to download from GCS.

Overriding parameters globally is accomplished at runtime using the Basic Override syntax provided by Hydra:

    ```python
    python3 download_process.py \
        dataset.data_dir={DATA_DIR} \
        dataset.label_map_path={LABEL_MAP_PATH} \
        dataset.size={SIZE}
    ```
Note that braces "{}" should be used to perform interpolation on Python variables.

See `configs/dataset/` for additional details on preconfigured values.
"""


from collections import namedtuple
from configs.dataclass.dataclasses import SSDResNet50Config
from configs.decorators.decorator import hydra_with_ray_remote
import google.protobuf
import hydra
from hydra import compose, initialize
from hydra.core.config_store import ConfigStore
import io
import logging
from object_detection.utils import dataset_util, label_map_util
from omegaconf import DictConfig, OmegaConf
import os
from PIL import Image
from psutil import cpu_count
import ray
import subprocess
import tensorflow as tf
from typing import List, TypeVar
from utils import bytes_list_feature, bytes_feature, float_list_feature
from utils import get_module_logger
from utils import int64_feature, int64_list_feature, parse_frame
from waymo_open_dataset import dataset_pb2 as open_dataset
from waymo_open_dataset import label_pb2


### Creating some not-so-long custom variable types for typing hints
RepeatedCompositeContainer = TypeVar(
        google.protobuf.pyext._message.RepeatedCompositeContainer
)
BoxLabel = TypeVar(waymo_open_dataset.label_pb2.Label.Box)
### Creating tuple format to store normalised bounding box coordinates
BboxTuple = namedtuple("BboxTuple", ['ymin', 'ymax', 'xmin', 'xmax'])
### Defining the ConfigStore and custom resolver to handle file path definitions
cs = ConfigStore.instance()
cs.store(name='model_config', node=SSDResNet50Config)
OmegaConf.register_new_resolver(
    "abspath", lambda relative_path: os.path.abspath(relative_path)
)


def _build_bounding_box(
        open_dataset_box: waymo_open_dataset.label_pb2.Label.Box, 
        image_width: int, image_height: int
) -> BboxTuple:
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
        xmax=min((center_x + (length / 2)) / image_width, 1.0))


def _get_label_from_id(label: int) -> str:
    """Returns the class label of the input class id."""
    return label_map_dict_inverted[label]


def create_tf_example(
    filename: str, encoded_jpeg: bytes, 
    annotations: RepeatedCompositeContainer, resize: bool=True
) -> tf.train.Example:
    """Converts to TensorFlow Object Detection API format.

    Annotations are assumed to be the bounding boxes and class labels
    captured by a single camera view.

    See: https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/using_your_own_dataset.md
   
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
        image_res = tf.cast(tf.image.resize(
                        image_tensor, size=(640, 640)), dtype=tf.uint8
        )
        encoded_jpeg = tf.io.encode_jpeg(image_res).numpy()
        width, height = 640, 640
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
    filename = filename.encode('utf8')
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


def download_tfr(file_path: str, data_raw_dir: str) -> str:
    """Download a single `.tfrecord` with `gsutil`.

    :param file_path: str, remote path to the `.tfrecord` file,
        this should start with 'gs://' and include the bucket name.
    :param data_raw_dir: str, the local path to the destination directory.
    returns: local_path (str), the absolute path to where the file is saved.
    """

    ### Get the file name from the absolute path
    file_name = os.path.basename(file_path)
    ### Define aboslute path to the downloaded `.tfrecord` file
    local_path = os.path.join(data_raw_dir, file_name)
    ### Return if the file has already been downloaded
    if os.path.exists(local_path):
        print('Skipping download of {}, file already exists!'.format(file_name))
        return local_path
    else:
        ### Download the `.tfrecord` file from GCS
        cmd = ['gcloud storage', 'cp', file_path, f'{local_path}']
        ### If running on Linux, use the following workaround:
        # Assumes that `google-cloud-sdk` is downloaded/unzipped into `addons` folder
        # Also that a user/service account has been authenticated/activated
        #sh_path = '/bin/sh'
        #gcloud_path = os.path.abspath(f'{data_raw_dir}/../../../addons/google-cloud-sdk')
        #gsutil_path = shutil.which('gsutil', path=os.path.join(gcloud_path, 'bin'))
        #cmd = [gsutil_path, 'cp', file_path, local_path]
        logger.info(f'Downloading {file_name}')
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if res.returncode != 0:
            logger.error(f'Could not download {file_path}')
    return local_path


def process_tfr(file_path: str, data_processed_dir: str):
    """Process a Waymo data file into a TF-compatible type.

    Formats the Waymo Open Data `.tfrecord` type into a 
    `tf.train.Example` type for use with the TF Object Detection API.

    :param file_path: Absolute path pointing to the `.tfrecord` to convert.
    :param data_processed_dir: Absolute path pointing to the destination directory.
    """

    # Get the name of file to convert
    file_name = os.path.basename(file_path)
    ### Create path to store processed file
    data_processed_path = os.path.join(data_processed_dir, file_name)
    ### Return if converted/processed file already exists
    if os.path.exists(data_processed_path):
        print('Skipping processing of {}, file already exists!'.format(file_name))
        return
    else:
        logging.info(f'Processing {file_path}')
        #writer = tf.python_io.TFRecordWriter(f'output/{file_name}')
        ### FIX: Updated in TensorFlow 2.0
        # Credit: https://github.com/datitran/raccoon_dataset/issues/90#issuecomment-647073794
        #writer = tf.compat.v1.python_io.TFRecordWriter(f'output/{file_name}')
        #writer = tf.io.TFRecordWriter(f'output/{file_name}')
        writer = tf.io.TFRecordWriter(data_processed_path)
        dataset = tf.data.TFRecordDataset(file_path, compression_type='')
        # For each Frame in the Waymo OD `.tfrecord`
        for idx, data in enumerate(dataset):
            frame = open_dataset.Frame()
            frame.ParseFromString(bytearray(data.numpy()))
            # Fetch the image and annotations in the Frame
            encoded_jpeg, annotations = parse_frame(frame)
            filename = file_name.replace('.tfrecord', f'_{idx}.tfrecord')
            tf_example = create_tf_example(
                            filename, encoded_jpeg, annotations, resize=True
            )
            ### Write the serialised tf.train.Example to the TFRecordDataset
            writer.write(tf_example.SerializeToString())
        writer.close()
    return


@ray.remote
def download_and_process(file_path: str, data_dir: str):
    """Downloads the requested file and converts them to TF-compatible format.

    Decorated with Ray to run asynchronously on separate Python workers.
    See: https://docs.ray.io/en/latest/ray-core/tasks.html#tasks

    :param file_path: the file path of the `.tfrecord` file to download from GCS,
        this should be a string starting with 'gs://' that also specifies the bucket name.
    :param data_dir: the absolute path to the local directory to store the downloaded file.
    """

    ### Create the 'raw' subdirectory for downloaded files
    data_raw_dir = os.path.join(data_dir, 'raw')
    os.makedirs(data_raw_dir, exist_ok=True)
    ### Create 'processed' subdirectory for output files
    data_processed_dir = os.path.join(data_dir, 'processed')
    os.makedirs(data_processed_dir, exist_ok=True)
    ### Re-import the logger for multiprocesing
    #logger = get_module_logger(__name__)
    ### Download the `.tfrecord` file and get its local path in the 'raw' subdirectory
    local_path = download_tfr(file_path, data_raw_dir)
    ### Process the `.tfrecord` into a TF API compatible format
    process_tfr(local_path, data_processed_dir)
    ### Delete the original `.tfrecord` file to save space
    logger.info(f'Deleting {local_path}')
    os.remove(local_path)


@hydra.main(version_base=None, config_path='../../configs', config_name='config.yaml')
def main(cfg: SSDResNet50Config):
    """Downloads and processes the remote GCS-hosted `.tfrecord` files.

    The following parameters can be modified at runtime:
        DATA_DIR:        str         Path to the `data` directory to download files to.
        LABEL_MAP_PATH:  str         Path to the dataset `label_map.pbtxt` file.
        SIZE:            int         Number of `.tfrecord` files to download from GCS.

    Overriding parameters globally at runtime is provided in the Hydra Basic Override syntax:
    ```python
    
    python3 download_process.py \
        dataset.data_dir={DATA_DIR} \
        dataset.label_map_path={LABEL_MAP_PATH} \
        dataset.size={SIZE}
    ```
    Note that braces "{}" should be used to perform interpolation on Python variables.

    See `configs/dataset/` for additional details on preconfigured values.
    """
    ### Initialise the logger instance
    logger = get_module_logger(__name__)
    ### Load the configuration
    #initialize(version_base=None, config_path='../../configs')
    #cfg = compose(config_name='config', return_hydra_config=True)
    cfg = OmegaConf.create(cfg)
    ### Fetching the Label Map file and building inverted label map dict
    label_map = label_map_util.load_labelmap(cfg['dataset'].label_map_path)
    label_map_dict = label_map_util.get_label_map_dict(label_map)
    label_map_dict_inverted = {
                cls_id:cls_label for cls_label, cls_id in label_map_dict.items()
    }    
    ### Opening the list of file paths to download from GCS with gsutil
    with open(os.path.join(cfg['dataset'].data_dir, 'filenames.txt'), 'r') as f:
        file_paths = f.read().splitlines()
    logger.info("Downloading {} files. Be patient, this will take a long time.".format(len(file_paths[:cfg['dataset'].size])))
    ### Initialise the a new local Ray instance
    ray.init(num_cpus=cpu_count())
    ### Process the batched data and store into '{data_dir}/processed' subdirectory
    workers = [download_and_process.remote(
                        fn, cfg['dataset'].data_dir) for fn in file_paths[:cfg['dataset'].size]]
    # Note that `SIZE` of batch is within object store memory limits
    # See: https://docs.ray.io/en/latest/ray-core/tasks/patterns/too-many-results.html#code-example
    _ = ray.get(workers)


if __name__ == "__main__":
    main()
