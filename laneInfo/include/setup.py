#from distutils.core import setup
from setuptools import setup

setup(name='line_detector',
	version='0.0.0',
	package_dir={'': '${CMAKE_CURRENT_SOURCE_DIR}'},
	packages=['line_detector'],
	install_requires=['opencv-python','numpy','duckie_utils'])