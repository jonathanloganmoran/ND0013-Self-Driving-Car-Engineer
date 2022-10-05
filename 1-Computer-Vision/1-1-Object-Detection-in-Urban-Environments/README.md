# Project 1.1: Object Detection in an Urban Environment
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From the Self-Driving Car Engineer Nanodegree programme offered at Udacity.


## File Descriptions

Filename                                                                            | Description
------------------------------------------------------------------------------------|--------------
`Exploratory-Data-Analysis/2022-09-29-Exploratory-Data-Analysis-Part-1.ipynb`       | Jupyter notebook file performing basic EDA on the processed Waymo perception dataset.
`Exploratory-Data-Analysis/2022-09-29-Exploratory-Data-Analysis-Part-2.ipynb`       | Jupyter notebook file looking at various class distributions, occlusions, etc.
`annotations/label_map.pbtxt`                                                       | StringIntLabelMap `proto` configuration file mapping class names (strings) to class IDs (integers).
`data/filenames.txt`                                                                | List of remote paths to every Waymo Open Dataset v1.2 segment (file) hosted on the Google Cloud Storage bucket.
`scripts/preprocessing/download_process.py`                                         | Python script to download and process the remote GCS files in `filenames.txt` as a `tf.data.Dataset` instance.
`scripts/preprocessing/create_splits.py`                                            | Python script to split Waymo data into training, validation and test sets.
`scripts/training/edit_config.py`                                                   | Python script to edit and save a new `TrainEvalPipelineConfig` Google Protobuf file.
`experiments/model_main_tf2.py`                                                     | Python script to create and run a TF-2 object detection model instance.
`experiments/exporter_main_v2.py`                                                   | Python script to export an object detection model for inference, configured with the `pipeline.config` file.


## Objectives
* Train an object detection model on the Waymo dataset;
* Learn how to use the TensorFlow object detection API;
* Learn how to pick the best parameters for your model;
* Perform an in-depth error analysis to understand your model's limitations. 


## Introduction
In this project you will apply the skills you have gained in this course to create a convolutional neural network to detect and classify objects using data from Waymo. You will be provided with a dataset of images of urban environments containing annotated cyclists, pedestrians 
and vehicles.

First, you will perform an extensive data analysis including the computation of label distributions, display of sample images, and checking for object occlusions. You will use this analysis to decide what augmentations are meaningful for this project. Then, you will train a 
neural network to detect and classify objects. You will monitor the training with TensorBoard and decide when to end it. Finally, you will experiment with different hyperparameters to improve your model's performance.

This project will include use of the TensorFlow Object Detection API, where you can deploy your model to get predictions on images sent to the API. You will also be provided with code to create a short video of the model predictions.


## TensorFlow Object Detection API
This API simplifies the training and development of object detection models in TensorFlow.


## Tasks
### Exploratory Data Analysis (EDA)
* ✅ Looking at sample data;
* ⬜️ Computation of label distributions;
* ⬜️ Checking for object occlusions;
* ⬜️ Deciding which data augmentation strategies to use;

### Model Training and Evaluation
* ⬜️ Defining the TF Object Detection API `config` file;
* ⬜️ Training a neural network;
* ⬜️ Monitoring with TensorBoard;
* ⬜️ Deciding when to end training;
* ⬜️ Classifying objects and evaluating performance;

### Improving Performance
* ⬜️ Manipulating hyperparameters;
* ⬜️ Revisiting data augmentation strategies.


#### Exercises
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
* [1] Sun, Pei, et al., Scalability in Perception for Autonomous Driving: Waymo Open Dataset. arXiv. 2019. [doi: 10.48550/ARXIV.1912.04838](https://arxiv.org/abs/1912.04838).

Helpful explanations:
* [Training Custom Object Detector by L. Vladimirov | TensorFlow 2 Object Detection API tutorial](https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/training.html)