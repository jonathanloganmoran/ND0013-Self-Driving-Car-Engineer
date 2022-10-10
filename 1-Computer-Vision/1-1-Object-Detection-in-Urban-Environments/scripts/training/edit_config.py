from configs.dataclass.dataclasses import SSDResNet50Config
import glob
from google.protobuf import text_format
import hydra
from hydra.core.config_store import ConfigStore
from object_detection.protos import pipeline_pb2
from omegaconf import DictConfig, OmegaConf
import os
import tensorflow.compat.v2 as tf


cs = ConfigStore.instance()
cs.store(name='model_config', node=SSDResNet50Config)
OmegaConf.register_new_resolver(
    "abspath", lambda relative_path: os.path.abspath(relative_path)
)


@hydra.main(version_base=None, config_path='../../configs', config_name='config.yaml')
def edit(cfg: SSDResNet50Config):
    """Creates a new config file.
    
    :param cfg: the Hydra config file specifying the dataset directory paths,
        label map path, the batch size and pre-trained checkpoint directory.

    The following parameters can be modified globally at runtime:
        TRAIN:                  str         Path to the `train` data directory.
        TEST:                   str         Path to the `test` data directory.
        VAL:                    str         Path to the `val` data directory.
        BATCH_SIZE:             int         Number of examples to process per iteration.
        CHECKPOINT_DIR:         str         Path to the pre-trained `checkpoint` folder.
        LABEL_MAP_PATH:         str         Path to the dataset `label_map.pbtxt` file.
        PIPELINE_CONFIG_PATH:   str         Path to the `pipeline.config` file to modify.
    
    Overriding parameters globally at runtime is provided in the Hydra Basic Override syntax:
    ```python
    
    python3 edit_config.py \
        dataset.train=TRAIN dataset.test=TEST dataset.val=VAL \
        dataset.label_map_path=LABEL_MAP_PATH \
        hyperparameters.batch_size=BATCH_SIZE \
        model.checkpoint_dir=CHECKPOINT_DIR \
        model.pipeline_config_path=PIPELINE_CONFIG_PATH
    ```
    
    See `configs/model/`, `configs/dataset/` and `configs/hyperparameters` for
    additional details on preconfigured values.
    """

    cfg = OmegaConf.create(cfg)
    pipeline_config = pipeline_pb2.TrainEvalPipelineConfig() 
    with tf.gfile.GFile(cfg['model'].pipeline_config_path, "r") as f:                                                                                                                                                                                                                     
        proto_str = f.read()                                                                                                                                                                                                                                          
        text_format.Merge(proto_str, pipeline_config)  
    training_files = glob.glob(cfg['dataset'].train + '/*.tfrecord')
    evaluation_files = glob.glob(cfg['dataset'].val + '/*.tfrecord')
    pipeline_config.train_config.batch_size = cfg['hyperparameters'].batch_size
    pipeline_config.train_config.fine_tune_checkpoint = cfg['model'].checkpoint_dir
    pipeline_config.train_input_reader.label_map_path = cfg['dataset'].label_map_path
    pipeline_config.train_input_reader.tf_record_input_reader.input_path[:] = training_files
    pipeline_config.eval_input_reader[0].label_map_path = cfg['dataset'].label_map_path
    pipeline_config.eval_input_reader[0].tf_record_input_reader.input_path[:] = evaluation_files
    config_text = text_format.MessageToString(pipeline_config)
    config_new_path = f"{os.path.dirname(cfg['model'].pipeline_config_path)}/pipeline_new.config"              
    with tf.gfile.Open(config_new_path, "wb") as f:                                                                                                                                                                                                                       
        f.write(config_text)   


if __name__ == "__main__":

    edit()