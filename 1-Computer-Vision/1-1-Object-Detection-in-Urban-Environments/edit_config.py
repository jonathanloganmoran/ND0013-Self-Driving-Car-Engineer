import argparse
import glob

import tensorflow.compat.v1 as tf
from google.protobuf import text_format
from object_detection.protos import pipeline_pb2


def edit(train_dir, eval_dir, batch_size, checkpoint, label_map):
    """
    edit the config file and save it to pipeline_new.config
    args:
    - train_dir [str]: path to train directory
    - eval_dir [str]: path to val OR test directory 
    - batch_size [int]: batch size
    - checkpoint [str]: path to pretrained model
    - label_map [str]: path to labelmap file
    """
    pipeline_config = pipeline_pb2.TrainEvalPipelineConfig() 
    with tf.gfile.GFile("pipeline.config", "r") as f:                                                                                                                                                                                                                     
        proto_str = f.read()                                                                                                                                                                                                                                          
        text_format.Merge(proto_str, pipeline_config)  
    
    training_files = glob.glob(train_dir + '/*.tfrecord')
    evaluation_files = glob.glob(eval_dir + '/*.tfrecord')

    pipeline_config.train_config.batch_size = batch_size
    pipeline_config.train_config.fine_tune_checkpoint = checkpoint
    pipeline_config.train_input_reader.label_map_path = label_map
    pipeline_config.train_input_reader.tf_record_input_reader.input_path[:] = training_files

    pipeline_config.eval_input_reader[0].label_map_path = label_map
    pipeline_config.eval_input_reader[0].tf_record_input_reader.input_path[:] = evaluation_files

    config_text = text_format.MessageToString(pipeline_config)             
    with tf.gfile.Open("pipeline_new.config", "wb") as f:                                                                                                                                                                                                                       
        f.write(config_text)   


if __name__ == "__main__": 
    parser = argparse.ArgumentParser(description='Download and process tf files')
    parser.add_argument('--train_dir', required=True, type=str,
                        help='training directory')
    parser.add_argument('--eval_dir', required=True, type=str,
                        help='validation or testing directory')
    parser.add_argument('--batch_size', required=True, type=int,
                        help='number of images in batch')
    parser.add_argument('--checkpoint', required=True, type=str,
                        help='checkpoint path')   
    parser.add_argument('--label_map', required=True, type=str,
                        help='label map path')   
    args = parser.parse_args()
    edit(args.train_dir, args.eval_dir, args.batch_size, 
         args.checkpoint, args.label_map)
    