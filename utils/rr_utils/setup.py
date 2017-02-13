#from distutils.core import setup
from setuptools import setup

setup(name='rr_utils',
	version='0.0.0',
	package_dir={'': '${CMAKE_CURRENT_SOURCE_DIR}'},
	packages=['rr_utils'],
	install_requires=['pyyaml','numpy','duckie_utils'])