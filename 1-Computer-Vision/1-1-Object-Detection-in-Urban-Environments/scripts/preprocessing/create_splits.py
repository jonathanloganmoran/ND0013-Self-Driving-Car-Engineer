from absl import app, flags, logging
import argparse
import glob
import numpy as np
import os
import random
import shutil
from utils import get_module_logger


def split(source: str, destination: str):
    """Create three splits from the processed records. 

    The files should be moved to new folders in the
    same directory. This folder should be named train, val and test.

    :param source: absolute path to the source data directory,
        contains the processed `.tfrecord` files to split.
    :param destination: absolute path to the destination data directory,
        contains the three subfolders: `train`, `val` and `test`.
    """
    # TODO: Implement function

    TRAIN_TEST_SPLIT = 0.8
    TRAIN_VAL_SPLIT = 0.8

    dirs = [os.path.join(destination, split) for split in ['train', 'test', 'val']]
    _ = [os.makedirs(d, exist_ok=True) for d in dirs]

    src = glob.glob(f"{source}/*.tfrecord")
    src = sorted(src, key=lambda x: random.random())
    train_set = src[:int(-len(src) * TRAIN_TEST_SPLIT)]
    test_set = src[int(-len(src) * TRAIN_TEST_SPLIT):]

    for file_path in train_set:
        file_name = os.path.basename(file_path)
        logger.info(f"Processing {file_name}")
        train_file = f"{destination}/train/{file_name}"
        val_file = f"{destination}/val/{file_name}"
        train_writer = tf.io.TFRecordWriter(train_file)
        val_writer = tf.io.TFRecordWriter(val_file)
        dataset = tf.data.TFRecordDataset(file_path, compression_type='')
        record_count = 0
        for record_count, _ in enumerate(dataset):
            # Get the number of records in dataset
            pass
        for i, data in enumerate(dataset):
            example = tf.train.Example()
            example.ParseFromString(data.numpy())
            if i < record_count * TRAIN_VAL_SPLIT:
                train_writer.write(example.SerializeToString())
            else:
                val_writer.write(example.SerializeToString())
        train_writer.close()
        val_writer.close()
    for file_path in test_set:
        file_name = os.path.basename(file_path)
        logger.info(f"Copying {file_name}")
        test_file = f"{destination}/test/{file_name}"
        shutil.move(file_path, test_file)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
                description='Split data into training / validation / testing')
    parser.add_argument('--source', required=True,
                help='source data directory')
    parser.add_argument('--destination', required=True,
                help='destination data directory')
    args = parser.parse_args()
    FLAGS = flags.FLAGS
    flags.DEFINE_string('source', args.source,
                'Absolute path to the data directory to split')
    flags.DEFINE_string('destination', args.destination,
                'Absolute path to the destination data directory to store splits')
    flags.mark_flag_as_required('source')
    flags.mark_flag_as_required('destination')
    logger = get_module_logger(__name__)
    logger.info('Creating splits...')
    split(FLAGS.source, FLAGS.destination)