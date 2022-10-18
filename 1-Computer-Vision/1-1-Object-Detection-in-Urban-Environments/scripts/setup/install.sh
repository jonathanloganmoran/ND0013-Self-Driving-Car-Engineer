### Installing the Waymo Open Dataset and TensorFlow Object Detection API ###
### This script should be ran and used in Google Colab ##
### By Jonathan L. Moran (jonathan.moran107@gmail.com) ##


### Cloning the Waymo Open Dataset repository
git clone https://github.com/waymo-research/waymo-open-dataset.git waymo-od
cd waymo-od && git branch -a
### Installing the Waymo Open Dataset API and dependencies
pip3 install --quiet --upgrade pip
pip3 install --quiet numpy==1.19.2
pip3 install --quiet waymo-open-dataset-tf-2-6-0==1.4.9
### Installing the components required for Bazel (v3.1.0)
sudo apt-get install --assume-yes pkg-config zip g++ zlib1g-dev unzip python3 python3-pip > /dev/null
wget https://github.com/bazelbuild/bazel/releases/download/3.1.0/bazel-3.1.0-installer-linux-x86_64.sh > /dev/null
sudo bash bazel-3.1.0-installer-linux-x86_64.sh > /dev/null
sudo apt install build-essential > /dev/null
### Installing the build requirements via Bazel
cd waymo-od && ./configure.sh && cat .bazelrc && bazel clean
### Building and testing with Bazel (can take ~10 min)
# !cd waymo-od && bazel build ... --show_progress_rate_limit=10.0
### Fetching the TF models/research/object_detection subdirectory
apt install subversion > /dev/null
svn checkout -q https://github.com/tensorflow/models/trunk/research/object_detection
### Installing Google Protobuf dependency
pip install protobuf > /dev/null
### Compiling the protobufs
pip install protobuf-compiler > /dev/null
protoc object_detection/protos/*.proto --python_out=.
### Installing the COCO API dependency
pip install cython > /dev/null
pip install pycocotools > /dev/null
cp object_detection/packages/tf2/setup.py .
pip install . > /dev/null
### Verifying installation was successful
python3 object_detection/builders/model_builder_tf2_test.py
### Patching tensorflow install to Waymo OD-compatible version (2.6.0)
pip install waymo-open-dataset-tf-2-6-0==1.4.9 > /dev/null


### To use in Google Colab, import the following ###

#### Importing the TensorFlow and Waymo Open Dataset APIs
# import google.protobuf
# import tensorflow as tf
# import waymo_open_dataset
# from waymo_open_dataset import dataset_pb2 as open_dataset
### Import the Waymo OD and TFDS Object Detection API utils
# import tensorflow as tf
# import google.protobuf
# from object_detection.utils import dataset_util, label_map_util
# from waymo_open_dataset import dataset_pb2, label_pb2
# from google.colab import auth
# auth.authenticate_user()