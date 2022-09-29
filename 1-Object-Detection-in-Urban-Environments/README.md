# Project 1.1: Object Detection in an Urban Environment
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From the Self-Driving Car Engineer Nanodegree programme offered at Udacity.


## File Descriptions

Filename                                           | Description
---------------------------------------------------|--------------
`2022-07-25-Exploratory-Data-Analysis.ipynb`       | A Jupyter notebook file performing EDA on the processed Waymo perception dataset.
`2022-07-25-Data-Augmentation-Strategies.ipynb`    | A Jupyter notebook file testing various data augmentation strategies. 
`experiments/model_main_tf2.py`                    | A Python script to create and run TF2 object detection models.
`experiments/exporter_main_v2.py`                  | A Python script to export an object detection model for inference.
`download_process.py`                              | A Python script to download and process the Waymo dataset from a Google Cloud bucket.
`create_splits.py`                                 | A Python script to split Waymo data into training, validation and test sets.
`edit_config.py`                                   | A Python script to edit and save a new config file using Google Protobuf and Object Detection API.


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
* ⬜️ Computation of label distributions;
* ⬜️ Looking at sample data;
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



## Credits
This assignment was prepared by Thomas Hossler and Michael Virgo et al., Winter 2021 (link [here](https://github.com/udacity/nd013-c1-vision-starter)).

Resources:
* [1] Waymo Open Dataset - Perception (https://waymo.com/open/);

Helpful explanations:
* []()
