### TensorFlow Object Detection API on Custom Dataset ### 
## By Jonathan L. Moran (jonathan.moran107@gmail.com) ###

# For more information on this script, see:
# https://setuptools.pypa.io/en/latest/references/keywords.html

from setuptools import setup, find_packages


reqs = parse_requirements('./build/requirements.txt', session=False)
install_requirements = [str(ir.req) for ir in reqs]


def readme():
	with open('./README.md') as f:
		return f.read()


setup(
	name='1-1-Object-Detection-in-Urban-Environments',
	version='1.0',
	description='2D Object Detection in Driving Environments',
	long_description=readme(),
	author='Jonathan L. Moran',
	author_email='jonathan.moran107@gmail.com',
	url='https://github.com/jonathanloganmoran/ND0013-Self-Driving-Car-Engineer/tree/main/1-Computer-Vision/1-1-Object-Detection-in-Urban-Environments',
	# Find and install Waymo Open Dataset API submodule in `addons/`
	packages=find_packages(),
	install_requires=install_requirements,
	include_package_data=True,

)
