# Lint as: python3
# Copyright 2020 The TensorFlow Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

r"""Creates and runs TF2 object detection models.

For local training/evaluation run:
PIPELINE_CONFIG_PATH=path/to/pipeline.config
MODEL_DIR=/tmp/model_outputs
NUM_TRAIN_STEPS=10000
SAMPLE_1_OF_N_EVAL_EXAMPLES=1
python model_main_tf2.py -- \
    --model_dir=$MODEL_DIR --num_train_steps=$NUM_TRAIN_STEPS \
    --sample_1_of_n_eval_examples=$SAMPLE_1_OF_N_EVAL_EXAMPLES \
    --pipeline_config_path=$PIPELINE_CONFIG_PATH \
    --alsologtostderr
"""

from configs.dataclass.dataclasses import SSDResNet50Config
import hydra
from hydra.core.config_store import ConfigStore
import tensorflow.compat.v2 as tf
from object_detection import model_lib_v2
import omegaconf
from omegaconf import DictConfig, OmegaConf


cs = ConfigStore.instance()
cs.store(name='model_config', node=SSDResNet50Config)
OmegaConf.register_new_resolver(
    "abspath", lambda relative_path: os.path.abspath(relative_path)
)



@hydra.main(version_base=None, config_path='../../configs', config_name='config.yaml')
def main(cfg: SSDResNet50Config):
    """Entry point to the TF Object Detection API model train loop.

    :param cfg: the Hydra config file specifying the required build
        parameters, model configuration variables and hyperparameters.

    The following parameters can be modified globally at runtime:
        DIR_BASE:                               str         Path to the current `model` subdirectory.
        MODEL_OUT:                              str         Path to the `/tmp/model_outputs` folder.
        CHECKPOINT_DIR:                         str         Path to the pretrained weights/variables saved in `checkpoint` folder.
        PIPELINE_CONFIG_PATH:                   str         Path to the `pipeline.config` file.
        NUM_TRAIN_STEPS:                        int         Number of training steps (batch iterations) to perform. 
        EVAL_ON_TRAIN_DATA:                     bool        If True, will evaluate on training data (only supported in distributed training).
        SAMPLE_1_OF_N_EVAL_EXAMPLES:            int         Number of evaluation samples to skip (will sample 1 of every n samples per batch).
        SAMPLE_1_OF_N_EVAL_ON_TRAIN_EXAMPLES:   int         Number of training samples to skip (only used if `eval_on_train_data` is True).
        EVAL_TIMEOUT:                           int         Number of seconds to wait for an evaluation checkpoint before exiting.
        USE_TPU:                                bool        Whether or not the job is executing on a TPU.
        TPU_NAME:                               str         Name of the Cloud TPU for Cluster Resolvers.
        CHECKPOINT_EVERY_N:                     int         Defines how often to checkpoint (every n steps).
        RECORD_SUMMARIES:                       bool        Whether or not to record summaries defined by the model or training pipeline.
        NUM_WORKERS:                            int         When `num_workers` > 1, training uses 'MultiWorkerMirroredStrategy',
                                                            When `num_workers` = 1, training uses 'MirroredStrategy'.

    using the Basic Override syntax provided by Hydra:
    ```python
    
    python3 model_main_tf2.py \
        model.dir_base={DIR_BASE} model.model_out={MODEL_OUT} \
        model.checkpoint_dir={CHECKPOINT_DIR} \
        model.pipeline_config_path={PIPELINE_CONFIG_PATH} \
        model.num_train_steps={NUM_TRAIN_STEPS} \
        model.sample_1_of_n_eval_examples={SAMPLE_1_OF_N_EVAL_EXAMPLES} \
        ...
    ```
    Note that braces "{}" should be used to perform interpolation on Python variables.

    See `configs/model/` for additional details on preconfigured values.
    """

    cfg = OmegaConf.create(cfg)
    tf.config.set_soft_device_placement(True)

    if FLAGS.checkpoint_dir:
        model_lib_v2.eval_continuously(
                pipeline_config_path=cfg['model'].pipeline_config_path,
                model_dir=cfg['model'].model_out,
                train_steps=cfg['model'].num_train_steps,
                sample_1_of_n_eval_examples=cfg['model'].sample_1_of_n_eval_examples,
                sample_1_of_n_eval_on_train_examples=(
                        cfg['model'].sample_1_of_n_eval_on_train_examples),
                checkpoint_dir=cfg['model'].checkpoint_dir,
                wait_interval=300, timeout=cfg['model'].eval_timeout)
    else:
        if cfg['model'].use_tpu:
            # TPU is automatically inferred if tpu_name is None and
            # we are running under cloud ai-platform.
            resolver = tf.distribute.cluster_resolver.TPUClusterResolver(
                    cfg['model'].tpu_name)
            tf.config.experimental_connect_to_cluster(resolver)
            tf.tpu.experimental.initialize_tpu_system(resolver)
            strategy = tf.distribute.experimental.TPUStrategy(resolver)
        elif cfg['model'].num_workers > 1:
            strategy = tf.distribute.experimental.MultiWorkerMirroredStrategy()
        else:
            strategy = tf.compat.v2.distribute.MirroredStrategy()

        with strategy.scope():
            model_lib_v2.train_loop(
                    pipeline_config_path=cfg['model'].pipeline_config_path,
                    model_dir=cfg['model'].model_out,
                    train_steps=cfg['model'].num_train_steps,
                    use_tpu=cfg['model'].use_tpu,
                    checkpoint_every_n=FLAGS.checkpoint_every_n,
                    record_summaries=cfg['model'].record_summaries)

if __name__ == '__main__':
    tf.compat.v1.app.run()