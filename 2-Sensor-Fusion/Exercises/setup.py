# https://setuptools.pypa.io/en/latest/references/keywords.html

from setuptools import setup, find_packages
from pip.req import parse_requirements


reqs = parse_requirements('./requirements.txt', session=False)
install_requirements = [str(ir.req) for ir in reqs]


setup(
    name='ND0013-Lesson-2-1-Lidar-Sensor-Exercises',
    version='0.1',
    description='Exercises from Lesson 1 of Course 2: Sensor Fusion in the Self-Driving Car Engineer Nanodegree programme.',
    author='Jonathan L. Moran',
    author_email='jonathan.moran107@gmail.com',
    # Installs the `simple_waymo_open_dataset_reader` package inside `tools/waymo_reader`
    packages=find_packages(),
    install_requires=install_requirements,
    include_package_data=True
)
