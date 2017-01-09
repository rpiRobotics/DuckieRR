#from distutils.core import setup
from setuptools import setup

setup(name='dagu_car',
	version='0.0.0',
	package_dir={'': '${CMAKE_CURRENT_SOURCE_DIR}/include'},
	packages=['dagu_car'],
	install_requires=['adafruit_drivers'])
