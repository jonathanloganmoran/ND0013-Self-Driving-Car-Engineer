# TensorFlow Object Detection API on Custom Dataset #
# Packaged with <3 by Jonathan L. Moran (jonathan.moran107@gmail.com).

r"""Function to split a set of files into train/test/validation subsets.

The files located in the `data_dir` are assumed to have been processed, e.g.,
converted into `tf.data.TFRecordDataset`. The `split()` function therefore only
calculates the split ratios and moves the files from their source directory into
the respective `split` subfolder.

 To split the downloaded data into three subsets `train`, `val`, and `test`, run:
    
    ```python
    python3 create_splits.py
    ```

    with none/any/all of the following:
        DATA_DIR:           str         Path to the source `data` directory.
        TRAIN:              str         Path to the `train` data directory.
        TEST:               str         Path to the `test` data directory.
        VAL:                str         Path to the `val` data directory.
        TRAIN_TEST_SPLIT:   float       Percent as [0, 1] to split train/test.
        TRAIN_VAL_SPLIT:    float       Percent as [0, 1] to split train/val.

    Overriding parameters globally is accomplished at runtime using the Basic Override syntax provided by Hydra:

    ```python
    python3 create_splits.py \
        dataset.data_dir={DATA_DIR} \
        dataset.train={TRAIN} dataset.test={TEST} dataset.val={VAL} \
        dataset.train_test_split={TRAIN_TEST_SPLIT} \
        dataset.train_val_split={TRAIN_VAL_SPLIT}
    ```
    Note that braces "{}" should be used to perform interpolation on Python variables.

    See `configs/dataset/` for additional details on preconfigured values.
"""


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

    In order to preserve disk space, the processed `tf.data.TFRecordDataset`
    files are moved from their source to the respective split folder.

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
        dataset.data_dir={DATA_DIR} \
        dataset.train={TRAIN} dataset.test={TEST} dataset.val={VAL} \
        dataset.train_test_split={TRAIN_TEST_SPLIT} \
        dataset.train_val_split={TRAIN_VAL_SPLIT}
    ```
    Note that braces "{}" should be used to perform interpolation on Python variables.

    
    See `configs/dataset/` for additional details on preconfigured values.
    """

    cfg = OmegaConf.create(cfg)
    dirs = [cfg['dataset'].train, cfg['dataset'].val, cfg['dataset'].test]
    _ = [os.makedirs(d, exist_ok=True) for d in dirs]
    src = glob.glob(f"{cfg['dataset'].data_dir}/processed/*.tfrecord")
    src = sorted(src, key=lambda x: random.random())
    ### Create splits by taking elements from left and right side
    test_start_idx = int(len(src) * cfg['dataset'].train_test_split)
    test_set = src[test_start_idx:]      # Get all files right of split
    train_set = src[:test_start_idx]     # Get all files left of split
    val_start_idx = int(len(train_set) * cfg['dataset'].train_val_split)
    ### Move processed `tf.data.TFRecordDataset` files to their `split` subfolder
    for src_file_path in train_set[:val_start_idx]
        train_file_name = os.path.basename(src_file_path)
        logger.info(f"Processing {train_file_name}")
        train_file_path = f"{cfg['dataset'].train}/{train_file_name}"
        shutil.move(src_file_path, train_file_path)
    for src_file_path in train_set[val_start_idx]:
        val_file_name = os.path.basename(src_file_path)
        logger.info(f"Processing {val_file_name}")
        val_file_path = f"{cfg['dataset'].val}/{val_file_name}"
        shutil.move(src_file_path, val_file_path)
    for src_file_path in test_set:
        test_file_name = os.path.basename(src_file_path)
        logger.info(f"Copying {test_file_name}")
        test_file_path = f"{cfg['dataset'].test}/{test_file_name}"
        shutil.move(src_file_path, test_file_path)


if __name__ == "__main__":
    split()