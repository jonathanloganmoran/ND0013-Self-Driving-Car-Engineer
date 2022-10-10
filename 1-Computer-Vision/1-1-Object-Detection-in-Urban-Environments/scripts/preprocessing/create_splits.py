from configs.dataclass.dataclasses import SSDResNet50Config
import glob
import hydra
from hydra.core.config_store import ConfigStore
import numpy as np
import omegaconf
from omegaconf import DictConfig, OmegaConf
import os
import random
import shutil
import tensorflow as tf
from utils import get_module_logger


cs = ConfigStore.instance()
cs.store(name='model_config', node=SSDResNet50Config)
OmegaConf.register_new_resolver(
    "abspath", lambda relative_path: os.path.abspath(relative_path)
)


@hydra.main(version_base=None, config_path='../../configs', config_name='config.yaml')
def split(cfg: SSDResNet50Config):
    """Create three splits from the processed records. 

    The three splits are `train`, `val` and `test`. Their relative sizes
    are defined inside the `configs/dataset/waymo_open_dataset.yaml` file.

    In order to preserve disk space, the test set files are moved from the
    source folder to the `test` split folder.

    :param cfg: the Hydra config file specifying the source and destination
        file paths and the train/test/val split sizes.

    The following parameters can be modified globally at runtime:
        DATA_DIR:           str         Path to the source `data` directory.
        TRAIN:              str         Path to the `train` data directory.
        TEST:               str         Path to the `test` data directory.
        VAL:                str         Path to the `val` data directory.
        TRAIN_TEST_SPLIT:   float       Percent as [0, 1] to split train/test.
        TRAIN_VAL_SPLIT:    float       Percent as [0, 1] to split train/val.

    using the Basic Override syntax provided by Hydra:
    ```python
    
    python3 create_splits.py \
        dataset.data_dir=DATA_DIR \
        dataset.train=TRAIN dataset.test=TEST dataset.val=VAL \
        dataset.train_test_split=TRAIN_TEST_SPLIT \
        dataset.train_val_split=TRAIN_VAL_SPLIT
    ```
    
    See `configs/dataset/` for additional details on preconfigured values.
    """

    cfg = OmegaConf.create(cfg)
    dirs = [cfg['dataset'].train, cfg['dataset'].val, cfg['dataset'].test]
    _ = [os.makedirs(d, exist_ok=True) for d in dirs]
    src = glob.glob(f"{cfg['dataset'].data_dir}/*.tfrecord")
    src = sorted(src, key=lambda x: random.random())
    train_set = src[:int(-len(src) * cfg['dataset'].train_test_split)]
    test_set = src[int(-len(src) * cfg['dataset'].train_test_split):]
    for file_path in train_set:
        file_name = os.path.basename(file_path)
        logger.info(f"Processing {file_name}")
        train_file = f"{cfg['dataset'].train}/{file_name}"
        val_file = f"{cfg['dataset'].val}/{file_name}"
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
            if i < record_count * cfg['dataset'].train_val_split:
                train_writer.write(example.SerializeToString())
            else:
                val_writer.write(example.SerializeToString())
        train_writer.close()
        val_writer.close()
    for file_path in test_set:
        file_name = os.path.basename(file_path)
        logger.info(f"Copying {file_name}")
        test_file = f"{cfg['dataset'].test}/{file_name}"
        shutil.move(file_path, test_file)


if __name__ == "__main__":

    split()