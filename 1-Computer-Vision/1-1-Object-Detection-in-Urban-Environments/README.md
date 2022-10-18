# Project 1.1: Object Detection in an Urban Environment
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From the Self-Driving Car Engineer Nanodegree programme offered at Udacity.


## File Descriptions

Filename                                                                            | Description
------------------------------------------------------------------------------------|--------------
`2022-10-11-Setup-and-Training.ipynb`                                               | Jupyter notebook file installing project dependencies in Linux environment; executes the project script files.
`Exploratory-Data-Analysis/2022-09-29-Exploratory-Data-Analysis-Part-1.ipynb`       | Jupyter notebook file performing basic EDA on the processed Waymo perception dataset.
`Exploratory-Data-Analysis/2022-09-29-Exploratory-Data-Analysis-Part-2.ipynb`       | Jupyter notebook file looking at various class distributions, occlusions, etc.
`setup.py`                                                                          | Sets the project base directory; used with `setuptools` to install project modules on build path.
`scripts/setup/install.sh`                                                          | Set of shell commands to install project dependencies within Google Colab.
`scripts/preprocessing/download_extract.py`                                         | Downloads and extracts attribute / bounding box data from the remote GCS files in `filenames.txt`.
`scripts/preprocessing/download_process.py`                                         | Downloads and processes the remote GCS files in `filenames.txt` as a `tf.data.Dataset` instance.
`scripts/preprocessing/create_splits.py`                                            | Splits the Waymo data into training, validation and test sets.
`scripts/data_analysis/visualisation.py`                                            | Set of functions to visualise the Waymo data and export Frame objects to GIF files.
`scripts/training/edit_config.py`                                                   | Edits and saves a new `TrainEvalPipelineConfig` Google Protobuf file.
`scripts/inference/inference_video.py`                                              | Runs inference and records the bounding box predictions to a GIF file.
`configs/dataset/waymo_open_dataset.yaml`                                           | The configuration file for the Waymo Open Dataset; only modify if necessary.
`configs/model/ssd_resnet50.yaml`                                                   | The configuration file for the RetinaNet pre-trained model; modify num. train steps and TPU settings as needed.
`configs/hyperparameters/hyperparameters1.yaml`                                     | The configuration file for the model hyperparameters (epochs, batch size, etc.).
`experiments/testing_configs.py`                                                    | Tests and verifies the Hydra configurations (replaces `absl` for CLI argument overriding).
`experiments/model_main_tf2.py`                                                     | Creates and runs a TF-2 object detection model instance.
`experiments/exporter_main_v2.py`                                                   | Exports an object detection model for inference; configured with the `pipeline.config` file.
`experiments/pretrained_model/ssd_resnet50_v1_fpn_640x640_coco17_tpu-8/`            | Pre-trained model subdirectory containing all relevant files and checkpoints e.g., `pipeline_new.config`.
`data/waymo_open_dataset/label_map.pbtxt`                                           | StringIntLabelMap `proto` configuration file mapping class names (strings) to class IDs (integers).
`data/waymo_open_dataset/filenames.txt`                                             | List of remote paths to every Waymo Open Dataset v1.2 segment (file) hosted on the Google Cloud Storage bucket.
`addons/`                                                                           | Subdirectory to store dependencies e.g., Google Cloud SDK, COCO API, Waymo Open Dataset API, etc.
`build/`                                                                            | Dockerfile and setup instructions for local environment configuration (not used).
`out/`                                                                              | Report figures, TensorBoard training/validation scalar charts, output detection predictions, etc.


## Objectives
* Train an object detection model on the Waymo dataset;
* Learn how to use the TensorFlow object detection API;
* Learn how to pick the best parameters for your model;
* Perform an in-depth error analysis to understand your model's limitations. 


## Introduction
In this project you will apply the skills you have gained in this course to create a convolutional neural network to detect and classify objects using data from the [Waymo Open Dataset](https://waymo.com/open) [1]. You will be provided with a dataset of images of urban environments containing annotated cyclists, pedestrians and vehicles.

First, you will perform an extensive data analysis including the computation of label distributions, display of sample images, and checking for object occlusions. You will use this analysis to decide what augmentations are meaningful for this project. Then, you will train a 
neural network to detect and classify objects. You will monitor the training with TensorBoard and decide when to end it. Finally, you will experiment with different hyperparameters to improve your model's performance.

This project will include use of the [TensorFlow Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) [2], where you can deploy your model to get predictions on images sent to the API. You will also be provided with code to create a short video of the model predictions.


## TensorFlow Object Detection API
The TensorFlow Object Detection API simplifies the development and training of state-of-the-art object detection models. A full list of pre-trained models can be viewed on the [TensorFlow Object Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md). 


## Setup and Installation
To configure your workspace, run:
```
pip install --editable {BASE_DIR}
```
where `BASE_DIR` points to the top-level project directory. All project modules will be installed onto your PYTHONPATH automatically.

To configure a Google Colab instance, copy the shell commands from [`scripts/setup/install.sh`]() into a cell and execute it. Note that with Google Colab instances there might be build issues when working with this project directory. It is only recommended to run individual scripts inside Google Colab.

If you are running this project inside the Linux Ubuntu LTS VM provided by Udacity, see build instructions in [`2022-10-16-Report.md`]().


## Data Acquisition

The following scripts will fetch and process the Waymo Open Dataset files from their Google Cloud Storage (GCS) hosted bucket. You will need to request access to the dataset ahead of time [here](https://waymo.com/open/licensing).

### Downloading and Processing

To download and process the `.tfrecord` files from Google Cloud Storage into `tf.data.TFRecordDataset` instances, run:

```python
python3 download_process.py
```

with none/any/all of the following parameters:
```
    DATA_DIR:        str         Path to the `data` directory to download files to.
    LABEL_MAP_PATH:  str         Path to the dataset `label_map.pbtxt` file.
    SIZE:            str         Number of `.tfrecord` files to download from GCS.
```

Overriding parameters globally is accomplished at runtime using the Basic Override syntax provided by Hydra:

```python
python3 download_process.py \
    dataset.data_dir=DATA_DIR \
    dataset.label_map_path=LABEL_MAP_PATH \
    dataset.size=SIZE
```
See [`configs/dataset/`]() for additional details on preconfigured values if running without parameters.


### Creating Dataset Splits

To split the downloaded data into three subsets `train`, `val`, and `test`, run:

```python
python3 create_splits.py
```

with none/any/all of the following:
```
    DATA_DIR:           str         Path to the source `data` directory.
    TRAIN:              str         Path to the `train` data directory.
    TEST:               str         Path to the `test` data directory.
    VAL:                str         Path to the `val` data directory.
    TRAIN_TEST_SPLIT:   float       Percent as [0, 1] to split train/test.
    TRAIN_VAL_SPLIT:    float       Percent as [0, 1] to split train/val.
```

Overriding parameters globally is accomplished at runtime using the Basic Override syntax provided by Hydra:

```python
python3 create_splits.py \
    dataset.data_dir=DATA_DIR \
    dataset.train=TRAIN dataset.test=TEST dataset.val=VAL \
    dataset.train_test_split=TRAIN_TEST_SPLIT \
    dataset.train_val_split=TRAIN_VAL_SPLIT
```
See [`configs/dataset/`]() for additional details on preconfigured values if running without parameters.


## Model configuration

In this project we will be using the RetinaNet (a SSD ResNet50 v1 with FPN) which has been pre-trained on the [COCO 2017](http://cocodataset.org/) dataset. First, fetch the default configuration file and pre-trained weights from the TensorFlow Object Detection Model Zoo, or alternatively from the [TensorFlow Hub](https://tfhub.dev/tensorflow/retinanet/resnet50_v1_fpn_640x640/1). 

In order to use this model on the Waymo Open Dataset, we have to re-configure some of the proto definitions in the [`pipeline.config`]() file (e.g., batch size, epochs, number of classes, etc.).


To configure the model for training, run:

```python
python3 edit_config.py
```

with none/any/all of the following parameters:
```
TRAIN:                                  str         Path to the `train` data directory.
TEST:                                   str         Path to the `test` data directory.
VAL:                                    str         Path to the `val` data directory.
BATCH_SIZE:                             int         Number of examples to process per iteration.
CHECKPOINT_DIR:                         str         Path to the pre-trained `checkpoint` folder.
LABEL_MAP_PATH:                         str         Path to the dataset `label_map.pbtxt` file.
PIPELINE_CONFIG_PATH:                   str         Path to the `pipeline.config` file to modify.
```

Overriding parameters globally is accomplished at runtime using the Basic Override syntax provided by Hydra:

```python
python3 edit_config.py \
    dataset.train=TRAIN dataset.test=TEST dataset.val=VAL \
    dataset.label_map_path=LABEL_MAP_PATH \
    hyperparameters.batch_size=BATCH_SIZE \
    model.checkpoint_dir=CHECKPOINT_DIR \
    model.pipeline_config_path=PIPELINE_CONFIG_PATH
```
See [`configs/model/`]() for additional details on preconfigured values if running without parameters.


## Training and Evaluation

For local training/evaluation run:

```python
python3 model_main_tf2.py
```

with none/any/all of the following parameters:
```
DIR_BASE:                               str         Path to the current `model` subdirectory.
MODEL_OUT:                              str         Path to the `/tmp/model_outputs` folder.
CHECKPOINT_DIR:                         str         Path to the pre-trained weights/variables saved in `checkpoint` folder (see note below).
PIPELINE_CONFIG_PATH:                   str         Path to the `pipeline_new.config` file (output from `edit_config.py`).
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
```

Overriding parameters globally is accomplished at runtime using the Basic Override syntax provided by Hydra (see note below):

```python
python3 model_main_tf2.py \
    model.pipeline_config_path=PIPELINE_CONFIG_PATH \
    model.model_out=MODEL_OUT model.num_train_steps=NUM_TRAIN_STEPS \
    model.sample_1_of_n_eval_examples=SAMPLE_1_OF_N_EVAL_EXAMPLES \
    ...
```
See [`configs/model/`]() for additional details on preconfigured values if running without parameters.

**NOTE**: for now, Hydra configuration support has been replaced with `argparse` command line arguments. This is due to an issue mentioned [here](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/issues/22). Hydra will be replacing the temporary `argparse` functionality in the `model_main_tf2.py` and `exporter_main_v2.py` scripts as soon as the bug as been resolved.


**NOTE**: the `CHECKPOINT_DIR` flag should only be included when running `model_main_tf2.py` for continuous evaluation alongside the training loop. In any case, the `CHECKPOINT_DIR` path included in the `edit_config.py` script should point to the _pre-trained model weights_, i.e., the `ckpt-0.data` and `ckpt-0.index` files downloaded from the TensorFlow Object Detection Model Zoo. The path to these checkpoint files inside the `checkpoint` folder should be  `/path/to/checkpoint/ckpt-0`. This isn't obvious upon first glance, since the path appears to reference `ckpt-0` as a subfolder of `checkpoint` when in fact it is not.


To export the trained model, run:

```python
python3 exporter_main_v2.py
```

with the following configuration parameters (see note on Hydra below):
```
input_type:                             object      Here we specify the `image_tensor` type.
PIPELINE_CONFIG_PATH:                   str         Path to the `pipeline_new.config` file (output from `edit_config.py`).
TRAINED_CHECKPOINT_DIR:                 str         Path to the training checkpoints, i.e., the `checkpoint` folder (see note below).
EXPORTED_MODEL_DIR:                     str         Path to the destination folder to store the saved model files.
```


```python 
python3 exporter_main_v2.py \
        --input_type image_tensor \
        --pipeline_config_path {PIPELINE_CONFIG_PATH} \
        --trained_checkpoint_dir {TRAINED_CHECKPOINT_DIR} \
        --output_directory {EXPORTED_MODEL_DIR} \
```

**NOTE**: The `TRAINED_CHECKPOINT_DIR` in `exporter_main_v2.py` differs from the one used in `model_main_tf2.py` above. This path should instead point to the folder where the _training_ checkpoints are saved, which should be inside the `MODEL_OUT` path configured above.


### Inference

To run inference over a `.tfrecord` file, run:

```python
python3 inference_video.py
```

with the following parameters provided as arguments to the command above (see note on Hydra below):
```
EXPORTED_MODEL_PATH:                    str         Path to the exported and saved model, i.e., the `model/saved_model` folder.
LABEL_MAP_PATH:                         str         Path to the dataset `label_map.pbtxt` file.
PIPELINE_CONFIG_PATH:                   str         Path to the `pipeline.config` file.
TF_RECORD_PATH:                         str         Path to the `.tfrecord` file to perform inference (object detection) over.
OUTPUT_PATH:                            str         Path to the output destination to save the inference video (a GIF file).
```

```python
python3 scripts/inference/inference_video.py \
	--labelmap_path {LABEL_MAP_PATH} \
	--model_path {EXPORTED_MODEL_PATH} \
	--config_path {PIPELINE_CONFIG_PATH}
	--tf_record_path {TF_RECORD_PATH} \
	--output_path {OUTPUT_PATH}
```

This script will export a video of the detection results on each frame of the `.tfrecord`. The video file is saved as a GIF.

**NOTE**: The `MODEL_PATH` should point to the folder where the _training_ checkpoints are saved. This path should be inside the `MODEL_OUT` folder and also be equivalent to `TRAINED_CHECKPOINT_DIR` configured above. 

**NOTE**: for now, Hydra configuration support has been replaced with `argparse` command line arguments. This is due to an issue mentioned [here](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/issues/22). Hydra will be replacing the temporary `argparse` functionality in the `inference_video.py` script as soon as the bug as been resolved.


## Tasks
### Exploratory Data Analysis (EDA)
* ✅ [Looking at sample data]();
* ✅ [Computation of label distributions]();
* ✅ [Checking for object occlusions / missing annotations]();
* ✅ [Deciding which data augmentation strategies to use]();

### Model Training and Evaluation
* ✅ [Defining the TF Object Detection API `pipeline.config` file]();
* ✅ [Training a neural network]();
* ✅ [Monitoring with TensorBoard]();
* ✅ [Deciding when to end training]();
* ✅ [Classifying objects and evaluating performance]();

### Improving Performance
* ⬜️ Manipulating hyperparameters;
* ⬜️ Revisiting data augmentation strategies.


### Exercises
* ✅ 1.1.1: [Choosing Metrics](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-1-1-Choosing-Metrics/2022-07-25-Choosing-Metrics-IoU.ipynb);
* ✅ 1.1.2: [Data Acquisition and Visualisation](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-1-2-Data-Acquisition-Visualisation/2022-08-01-Data-Acquisition-Visualisation.ipynb);
* ✅ 1.1.3: [Creating TensorFlow TFRecords](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-1-3-Creating-TF-Records/2022-08-03-Creating-TF-Records.ipynb);
* ✅ 1.2.1: [Camera Calibration and Distortion Correction](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-2-1-Calibration-Distortion/2022-08-10-Calibration-Distortion-Correction.ipynb);
* ✅ 1.2.2: [Image Manipulation and Masking](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-2-2-Image-Manipulation/2022-08-17-Image-Manipulation-Masking.ipynb);
* ✅ 1.2.3: [Geometric Transformations](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-2-3-Geometric-Transformations/2022-08-23-Geometric-Transformations-Image-Augmentation.ipynb);
* ✅ 1.3.1: [Logistic Regression](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-3-1-Logistic-Regression/2022-08-27-Logistic-Regression.ipynb);
* ✅ 1.3.2: [Stochastic Gradient Descent](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-3-2-Stochastic-Gradient-Descent/2022-08-29-Stochastic-Gradient-Descent.ipynb);
* ✅ 1.3.3: [Image Classification with Feedforward Neural Networks (FNNs)](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-3-3-Image-Classification-FNNs/2022-09-05-Image-Classification-Feed-Forward-Neural-Networks.ipynb);
* ✅ 1.4.1: [Pooling Layers in Convolutional Neural Networks (CNNs)](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-4-1-Pooling-Layers-CNNs/2022-09-07-Pooling-Layers-Convolutional-Neural-Networks.ipynb);
* ✅ 1.4.2: [Building Custom Convolutional Neural Networks (CNNs)](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-4-2-Building-Custom-CNNs/2022-09-12-Building-Custom-Convolutional-Neural-Networks.ipynb);
* ✅ 1.4.3: [Image Augmentations for the Driving Domain](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-4-3-Image-Augmentations/2022-09-19-Image-Augmentations.ipynb);
* ✅ 1.5.1: [Non-Maximum Suppression (NMS) and Soft-NMS](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-5-1-Non-Maximum-Suppression/2022-09-21-Non-Maximum-Suppression.ipynb);
* ✅ 1.5.2: [Mean Average Precision (mAP)](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-5-2-Mean-Average-Precision/2022-09-25-Mean-Average-Precision.ipynb);
* ✅ 1.5.3: [Learning Rate Schedules and Adaptive Learning Rate methods](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/main/1-Object-Detection-in-Urban-Environments/Exercises/1-5-3-Learning-Rate-Schedules/2022-09-28-Learning-Rate-Schedules.ipynb). 


## Credits
This assignment was prepared by Thomas Hossler and Michael Virgo et al., Winter 2021 (link [here](https://github.com/udacity/nd013-c1-vision-starter)).

Resources:
* [1] Huang, J. et al., Speed/accuracy trade-offs for modern convolutional object detectors. Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR). 2017. [doi: 10.48550/arXiv.1611.10012](https://arxiv.org/abs/1611.10012).
* [2] Sun, Pei, et al., Scalability in Perception for Autonomous Driving: Waymo Open Dataset. arXiv. 2019. [doi: 10.48550/ARXIV.1912.04838](https://arxiv.org/abs/1912.04838).

Helpful explanations:
* [Training Custom Object Detector by L. Vladimirov | TensorFlow 2 Object Detection API tutorial](https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/training.html);
* [Waymo Open Dataset Document by @Jossome | GitHub](https://github.com/Jossome/Waymo-open-dataset-document)