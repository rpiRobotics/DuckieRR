#from distutils.core import setup
from setuptools import setup

setup(name='duckie_utils',
	version='0.0.1',
	package_dir={'': '${CMAKE_CURRENT_SOURCE_DIR}'},
	packages=['duckie_utils'],
	install_requires=['pyyaml','numpy'])