#from distutils.core import setup
from setuptools import setup

setup(name='adafruit_drivers',
	version='0.0.0',
	package_dir={'': '${CMAKE_CURRENT_SOURCE_DIR}/include'},
	packages=['Adafruit_ADS1x15', 'Adafruit_GPIO','Adafruit_I2C','Adafruit_MotorHAT','Adafruit_PWM_Servo_Driver'],
	)
