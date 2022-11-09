# Project 2.1: 3D Object Detection with LiDAR Data
#### By Jonathan L. Moran (jonathan.moran107@gmail.com)
From Course 2: Sensor Fusion in the Self-Driving Car Engineer Nanodegree programme offered at Udacity.


## Objectives
* Write functions to load and pre-process 3D LiDAR point clouds from the [Waymo Open Dataset](https://waymo.com/open);
* Implement several deep learning-based models to detect and classify objects in 3D;
* Evaluate and visualise the detection results.


## Introduction
In this project you will apply the skills you have gained in this course to implement a deep neural network to detect and classify objects in 3D using LiDAR point cloud data from the [Waymo Open Dataset](https://waymo.com/open) [1]. You will be provided with a dataset of range images capturing the driving scenes of a suburban environment. Each range image comes with 3D annotated bounding boxes for the four object classes — cyclists, pedestrians, signs and vehicles. We will be evaluating the performance of two popular deep learning architectures on the 3D object detection task over this dataset.

<img src="figures/midterm_report/2022-11-06-Figure-8-Range-Images-Various-Scenes.gif" width="70%" height="70%" alt="Figure 1. Range images — visualising three segment files from the Waymo Open Dataset.">
</a>

$$
\begin{align}
\textrm{Figure 1. Range images — visualising three segment files from the Waymo Open Dataset.} \\
\end{align}
$$


First, you will perform an extensive data analysis including the visualisation of LiDAR point clouds. Using the Open3D library [2] you will analyse the point clouds for vehicle visibility and object occlusions and use these findings to obtain scenes of interest. Then you will perform data cleaning on the LiDAR range images and compare two normalisation techniques. You will use this analysis to decide what pre-processing steps are meaningful for this project. Furthermore, you must indentify vehicle landmarks that have a consistent appearance across frames. Lastly, you will implement two deep learning networks to detect and classify objects in 3D. You will evaluate and compare the performance of these models on the detection task using the COCO2017 evaluation metrics.

<img src="figures/midterm_report/2022-11-06-Figure-10-Detections-in-BEV-Shown-with-LiDAR-PCL-1.png" width="70%" height="70%" alt="Figure 2. LiDAR point cloud and bounding box annotations — results from the SFA3D model, a side-by-side comparison of the 3D point cloud image alongside the RGB-and-BEV images and corresponding bounding box annotations.">
</a>

$$
\begin{align}
\textrm{Figure 2. LiDAR point cloud and bounding box annotations — results from the SFA3D model.} \\
\end{align}
$$

The object detection results from the SFA3D model are shown in (b) from the above figure. The 3D ground-truth bounding box annotations are shown in green in the upper-half of (b) alongside the predictions, in red, projected into the BEV image space in the lower-half of (b).


This project will include use of the [SFA3D](https://github.com/maudzung/SFA3D) model [3], a pre-trained object detection network with ResNet-18 Keypoint Feature Pyramid Network (KPFN) [4] backbone. The SFA3D model performance will be compared to the DarkNet architecture, a Complex-YOLO [5] model for 3D object detection with Euler angle regression for bounding box heading angle predictions.


## File Descriptions

Filename                                                             | Description
---------------------------------------------------------------------|----------------
`2022-11-06-Project-Writeup.md`                                      | Markdown-formatted report of the key findings and evaluation results from this project.
`setup.py`                                                           | Builds the project, installs dependencies and places all modules onto the PYTHONPATH via `pip install -e` command (see below).
`loop_over_dataset.py`                                               | Entry point to the programme, implements all functions needed to parse, preview, modify, and evaluate on the LiDAR range images.
`data/filenames.txt`                                                 | List Google Cloud Storage (GCS) file paths to the `.tfrecord` used for model evaluation.
`student/objdet_detect.py`                                           | Performs 3D object detection; loads model configs, initialises PyTorch models, runs detection loop.
`student/objdet_eval.py`                                             | Performs performance assessment of the object detection model; computes IoU, position errors, precision / recall scores.
`student/objdet_pcl.py`                                              | Parses and modifies LiDAR point cloud data; pre-processes and converts range images to PCL; creates BEV maps from PCL.
`misc/evaluation.py`                                                 | Plotting functions; tracking, visualisation, and RMSE calculations.
`misc/helpers.py`                                                    | Helper functions; saves / loads binary files, creates execution list.
`misc/objdet_tools.py`                                               | Object detection helper functions; computes beam inclinations, converts to polar / Cartesian coordinates, computes bbox conversions, extracts rotation matrices; projects PCL / detections / ground-truth labels into BEV map / RGB image in vehicle coordinate system.


## Setup and Installation
To configure your workspace, run:
```
pip install --editable {BASE_DIR}
```
where `BASE_DIR` points to the top-level project directory. All project modules will be installed onto your PYTHONPATH automatically.


## Data Acquisition

This project relies on three `.tfrecord` files from the Waymo Open Dataset for data analysis, pre-processing, inference and evaluation. These filenames are referenced in `data/waymo_open_dataset/filenames.txt`.

See [Project 1.1: Object Detection in Urban Environments](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/1-Computer-Vision/1-1-Object-Detection-in-Urban-Environments) for information on downloading and processing the Waymo Open Dataset files from their Google Cloud Storage (GCS) hosted bucket. You will need to request access to the dataset ahead of time [here](https://waymo.com/open/licensing).

## Tasks


### Exploratory Data Analysis (EDA)

See [`2022-11-06-Project-Writeup.md`]() for an in-depth look at the LiDAR range image data provided in the Waymo Open Dataset. In the report we discuss our findings regarding vehicle landmarks and scenes of interest.

### Data Pre-Processing

This project consists of three major pre-processing tasks: visualisation, data manipulation and conversion. Covered in the [`2022-10-20-Course-2-Sensor-Fusion-Exercises-Part-1.ipynb`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-1-3D-Object-Detection/2-Sensor-Fusion/Exercises/2022-10-20-Course-2-Sensor-Fusion-Exercises-Part-1.ipynb) notebook are the functions used to visualise the LiDAR range images, normalise their intensity and range channel values, and build 3D point clouds and BEV map images from the range images. An explanation of how each function can be executed in this project can be found in [`2022-11-06-Project-Writeup.md`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-1-3D-Object-Detection/2-Sensor-Fusion/2-1-3D-Object-Detection-with-LiDAR-Data/2022-11-06-Project-Writeup.md).


### Model Inference and Evaluation

In this project we use the Super Fast and Accurate 3D Object Detection Based on 3D LiDAR Point Clouds (SFA3D) [3] model. The SFA3D model relies on a ResNet-18 FPN backbone [4]. The final model was trained for 300 epochs on the 3D KITTI dataset [6]. All model code and weights can be forked from the GitHub repository by [@maudzung](https://github.com/maudzung/SFA3D).

We also make use of the pre-trained DarkNet (Complex-YOLO) [5] model in this project. Its implementation in PyTorch is made available on GitHub by user [@](). A comparison of the performance between these two models is coming soon.


### Configuring the programme

First, fetch the pre-trained weights and model code from the GitHub repositories linked above. Next, ensure that all project requirements / dependencies have been installed (see _Setup and Installation_ or refer to `requirements.txt` and the `tools/` folder).

Note that if using the Udacity Ubuntu 16.04.6 LTS VM, the Matplotlib backend must be specified in order to avoid a `Segmentation fault (core dumped)` error (see [Issue #27](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/issues/27)). Currently, I am able to circumvent this by setting Matplotib to a non-GUI backend with `matplotlib.use('agg')`. This approach assumes that Matplotlib plotting functions will *not* be used, i.e., [`plt.show()`](https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.show.html) or [`plt.imshow()`](https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.imshow.html) are replaced with [`plt.savefig()`](https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.savefig.html).

#### Initialising the parameters

In order to make use of this programme, several parameters must be modified inside the `loop_over_dataset.py` file. These include:
```
MODEL_NAME:                             str         The desired pre-trained model to evaluate (can be either 'darknet' or 'fpn_resnet').
data_filename:                          str         Name of the downloaded Waymo Open Dataset `.tfrecord` file to use (see `filenames.txt`).
show_only_frames:                       List[int]   Range of frames from the `.tfrecord` file to look at (must be integers in format: `[stop, start]`).
exec_data:                              List[str]   List of data manipulation functions to execute.
exec_detection:                         List[str]   List of object detection functions to execute.
exec_tracking:                          List[str]   List of multi-target tracking functions to execute (not used in this project).
exec_visualization:                     List[str]   List of visualisation functions to execute.
```

Once the above have been initialised, run:

```python
python3 loop_over_dataset.py
```

Currently, all configuration settings for the `'darknet'` or `'fpn_resnet'` models must be manually-specified inside the [`student/objdet_detect.py`](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-1-3D-Object-Detection/2-Sensor-Fusion/2-1-3D-Object-Detection-with-LiDAR-Data/student/objdet_detect.py) file. Migration to a command-line based configuration library, e.g., [Hydra](https://hydra.cc), is planned.


#### Performing Inference and Evaluation

To run inference and plot the detection performance measures over a desired `.tfrecord` file, edit the following parameters with:

```python
exec_data = ['pcl_from_rangeimage', 'load_image']
exec_detection = ['bev_from_pcl', 'detect_objects']
exec_tracking = []
exec_visualization = ['show_objects_in_bev_labels_in_camera']
```

and specify the `MODEL_NAME` you wish to use in the inference loop. This will return an OpenCV window for each frame containing the BEV map and RGB image annotated with the predicted and ground-truth 3D bounding boxes.

You may also override the `MODEL_NAME` to load for inference with the following line:

```python
configs_det = det.load_configs(model_name={MODEL_NAME})
```

In order to use a custom pre-trained model (i.e., not the `'fpn_resnet'` or `'darknet'` model), you must modify the following functions inside `student/objdet_detect.py`:
* `detect_objects`: Performs inference and post-processing of the object detections;
* `load_configs_model`: Loads the model ocnfigurations into an `EasyDict` instance;
* `create_model`: Builds and returns a [`torch.nn.Module`](https://pytorch.org/docs/stable/_modules/torch/nn/modules/module.html#Module) instance from the `EasyDict` configuration settings.

Note that implementing a custom model assumes that you have (a) modified the functions above to support loading / configuring the model, (b) placed the PyTorch-implemented model files inside `tools/objdet_models` and (c) have saved the serialised detections and ground-truth bounding box annotations, LiDAR BEV images and LiDAR point clouds required to make use of the [`objdet_eval.py`]() scripts as well as the visualisation functions inside [`loop_over_dataset.py`](). Already configured in this project is support for the SFA3D model which may serve as an example for the devoted engineer.

## Tasks
### Exploratory Data Analysis (EDA)
* ✅ [Understanding LiDAR data in the Waymo Open Dataset]();
* ✅ [Extracting and visualising range images]();
* ✅ [Normalising the intensity and range channel values]();
* ✅ [Finding vehicle landmarks and scenes of interest]();
* ✅ [Converting range images to 3D point clouds]();
* ✅ [Converting 3D point clouds to BEV map images]();

### Model Training and Evaluation
* ✅ [Configuring and loading the object detection models]();
* ✅ [Classifying objects in 3D with the SFA3D model]();
* ✅ [Evaluating the detection performance of the SFA3D model]();
* ⬜️ Evaluating the detection performance of the DarkNet model;

### Improving Performance
* ⬜️ Fine-tuning the detection nets on the Waymo Open Dataset.


## Credits
This assignment was prepared by Dr. Andreas Haja and Dr. Antje Muntzinger et al., 2021 (link [here](https://github.com/udacity/nd013-c2-fusion-starter)).

References
* [1] Sun, Pei, et al. Scalability in Perception for Autonomous Driving: Waymo Open Dataset. arXiv. 2019. [doi:10.48550/ARXIV.1912.04838](https://arxiv.org/abs/1912.04838).
* [2] Zhou, Q-Y, et al. Open3D: A Modern Library for 3D Data Processing. arXiv. 2018. [doi:10.48550/ARXIV.1801.09847](https://arxiv.org/abs/1801.09847).
* [3] Peixuan, L., et al. RTM3D: Real-time Monocular 3D Detection from Object Keypoints for Autonomous Driving. arXiv. 2020. [doi:10.48550/arXiv.2001.03343](https://arxiv.org/abs/2001.03343).
* [4] He, K., et al. Deep Residual Learning for Image Recognition. IEEE Conference on Computer Vision and Pattern Recognition (CVPR). 2016. [doi:10.1109/CVPR.2016.90](https://doi.org/10.1109/CVPR.2016.90).
* [5] Simon, M., et al. Complex-YOLO: Real-time 3D Object Detection on Point Clouds. arXiv. 2018. [doi:10.48550/arXiv.1803.06199](https://arxiv.org/abs/1803.06199).
* [6] Geiger, A., et al. Vision Meets Robotics: The KITTI Dataset. International Journal of Robotics Reearch, 32(11):1231-1237. 2013. [doi:10.1177/0278364913491297](https://doi.org/10.1177/0278364913491297).

Helpful explanations:
* [`Technical_details.md` by @maudzung | GitHub](https://github.com/maudzung/SFA3D/blob/0e2f0b63dc4090bd6c08e15505f11d764390087c/Technical_details.md);
* [`2022-10-20-Course-2-Sensor-Fusion-Exercises-Part-1.ipynb` by Jonathan L. Moran | GitHub](https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/2-1-3D-Object-Detection/2-Sensor-Fusion/Exercises/2022-10-20-Course-2-Sensor-Fusion-Exercises-Part-1.ipynb);
* [`2022-11-06-Project-Writeup.md`] by Jonathan L. Moran | GitHub(https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/blob/a330864a997eb63caebe38e1f5c23c6f183796ff/2-Sensor-Fusion/2-1-3D-Object-Detection-with-LiDAR-Data/2022-11-06-Project-Writeup.md).