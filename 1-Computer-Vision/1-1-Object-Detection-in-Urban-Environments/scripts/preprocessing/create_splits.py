from absl import app, flags, logging
import argparse
import glob
import numpy as np
import os
import random
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