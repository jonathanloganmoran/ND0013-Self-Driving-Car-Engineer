# ------------------------------------------------------------------------------
# Project "3D Object Detection with LiDAR Data"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com)
#
# Purpose of this file : Process and prepare point cloud for object detection.
#
# You should have received a copy of the Udacity license with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
#
# NOTE: The current version of this programme relies on Numpy to perform data 
#       manipulation, however, a platform-specific implementation, e.g.,
#       TensorFlow `tf.Tensor` data ops, is recommended.
# ------------------------------------------------------------------------------

# https://setuptools.pypa.io/en/latest/references/keywords.html

from setuptools import setup, find_packages
from pip.req import parse_requirements


reqs = parse_requirements('./requirements.txt', session=False)
install_requirements = [str(ir.req) for ir in reqs]


setup(
    name='ND0013-Project-2-1-3D-Object-Detection-with-LiDAR-Data',
    version='1.0',
    description='Project 2.1: 3D Object Detection with LiDAR Data from Course 2: Sensor Fusion in the Self-Driving Car Engineer Nanodegree programme.',
    author='Jonathan L. Moran',
    author_email='jonathan.moran107@gmail.com',
    # Installs the `simple_waymo_open_dataset_reader` package inside `tools/waymo_reader`
    packages=find_packages(),
    install_requires=install_requirements,
    include_package_data=True
)
