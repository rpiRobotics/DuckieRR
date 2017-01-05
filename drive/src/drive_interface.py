#!/usr/bin/env python
from dagu_car.dagu_wheels_driver import DaguWheelsDriver

import argparse
import numpy as np
import socket
import RobotRaconteur as RR

RRN = RR.RobotRaconteurNode.s


drive_servicedef="""
#Service to provide interface to Duckiebot wheels
service Duckiebot_Interface

option version 0.8

object Duckiebot

property uint8 eStop
property double gain
property double trim
property double baseline
property double radius
property double k
property double limit


function void wheelCmd(double vL, double vR)
function void carCmd(double v, double omega)
function void toggleEStop()
function void resetParams()

end object

"""

class WheelsDriver_impl(object):
	def __init__(self):
		self.nodeName = "drive"
		self.driver = DaguWheelsDriver()

		self.default_params = {'gain':1.0, 
							   'trim':0.0, 
							   'baseline':0.1,
							   'radius':0.0318,
							   'k':27.0,
							   'limit':1.0}

		self._gain = self.default_params['gain']
		self._trim = self.default_params['trim']
		self._baseline = self.default_params['baseline']
		self._radius = self.default_params['radius']
		self._k = self.default_params['k']
		self._limit = self.default_params['limit']

		self.limit_max = 1.0
		self.limit_min = 0.0

		self._estop = False

	def wheelCmd(self, vL, vR):
		if self._estop:
			self.driver.setWheelsSpeed(left=0.0, right=0.0)
			return
		self.driver.setWheelsSpeed(left=vL, right=vR)

	def carCmd(self, v, omega):
		if self._estop:
			self.driver.setWheelsSpeed(left=0.0, right=0.0)
			return
		# implement the INVERSE KINEMATICS to determine the left and right wheel velocities
		# assuming same motor constants k for both motors
		k_r = self._k
		k_l = self._k

		# adjusting k by gain and trim
		k_r_inv = (self._gain + self._trim) / k_r
		k_l_inv = (self._gain - self._trim) / k_l

		omega_r = (v + 0.5 * omega * self._baseline) / self._radius
		omega_l = (v - 0.5 * omega * self._baseline) / self._radius

		# convert from motor rotation rate to duty cycle
		# u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        # send the speed to the wheels
        self.driver.setWheelsSpeed(left=u_r_limited, right=u_r_limited)

	def toggleEStop(self):
		self._estop = not self._estop
		if self._estop:
			print "[%s] Emergency Stop Activated"%(self.nodeName)
		else:
			print "[%s] Emergency Stop Released"%(self.nodeName)

	def resetParams(self):
		self._gain = self.default_params['gain']
		self._trim = self.default_params['trim']
		self._baseline = self.default_params['baseline']
		self._radius = self.default_params['radius']
		self._k = self.default_params['k']
		self._limit = self.default_params['limit']

	@property
	def eStop(self):
		if self._estop:
			return 1
		else:
			return 0

	@property
	def gain(self):
		return self._gain
	@gain.setter
	def gain(self,value):
		self._gain = value

	@property
	def trim(self):
		return self._trim
	@trim.setter
	def trim(self,value):
		self._trim = value

	@property
	def baseline(self):
		return self._baseline
	@baseline.setter
	def baseline(self,value):
		self._baseline = value

	@property
	def radius(self):
		return self._radius
	@radius.setter
	def radius(self,value):
		self._radius = value

	@property
	def k(self):
		return self._k
	@k.setter
	def k(self,value):
		self._k = value

	@property
	def limit(self):
		return self._limit
	@limit.setter
	def limit(self,value):
		try:
			if value > self.limit_max:
				raise ValueError('Warning: Max limit is 1.0')
			elif value < self.limit_min:
				raise ValueError('Warning: Min limit is 0.0')
		except ValueError:
			pass
		finally:
			self._limit = max(min(value,self.limit_max),self.limit_min)

		
	def on_shutdown(self):
		self.driver.setWheelsSpeed(left=0.0, right=0.0)
		print "[%s] Shutting Down"%(self.nodeName)


if __name__ == '__main__':
	# Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Initialize the Duckiebot')
    parser.add_argument('--port',type=int,default=0,
        help='TCP port to host service on' +\
        '(will auto-generate if not specified)')
    #parser.add_argument('--veh', required=True,
        #help='The name of the duckiebot being launched')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])
    #veh = args.veh
	
	# Enable Numpy
	RRN.UseNumPy = True # NECESSARY?

	# Create Local Transport, start server as name, and register it
	t1 = RR.LocalTransport()
	t1.StartServerAsNodeName("DuckiebotServer.Drive")
	RRN.RegisterTransport(t1)

	# Create TCP Transport, register it, and start the server  
	t2 = RR.TcpTransport()
	t2.EnableNodeAnnouce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
			RR.IPNodeDiscoveryFlags_LINK_LOCAL |
			RR.IPNodeDiscoveryFlags_SITE_LOCAL)

	RRN.RegisterTransport(t2)

	port = args.port
	t2.StartServer(port)
	if (port == 0):
		port = t2.GetListenPort()


	# Initialize the object
	driver_obj = WheelsDriver_impl()
	
	# Register the service def
	RRN.RegisterServiceType(drive_servicedef)

	# Register the service
	RRN.RegisterService("Duckiebot","Duckiebot_Interface/Duckiebot", driver_obj)

	print "Service started, connect via one of the following:"
	print "rr+local:///?nodename=DuckiebotServer.Drive&service=Duckiebot"
	print "rr+tcp://localhost:%s/?service=Duckiebot"%(port)
	print "rr+tcp://localhost:%s/?nodename=DuckiebotServer.Drive&service=Duckiebot"%(port)
	print "rr+tcp://%s.local:%s/?nodename=DuckiebotServer.Drive&service=Duckiebot"%(socket.gethostname(), port)
	print "rr+tcp://%s:%s/?nodename=DuckiebotServer.Drive&service=Duckiebot"%(socket.gethostbyname(socket.gethostname()), port)
	try:
		while True:

	except (KeyboardInterrupt,SystemExit):
		driver_obj.on_shutdown()
		
		# This must be here to prevent segfault
		RRN.Shutdown()
		sys.exit(0)