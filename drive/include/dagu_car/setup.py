#from distutils.core import setup
from setuptools import setup

setup(name='dagu_car',
	version='0.0.0',
	package_dir={'': '/home/greg/duckiebot_ws/DuckieRR/drive/include/dagu_car/include'},
	packages=['dagu_car'],
	install_requires=['adafruit_drivers'])
