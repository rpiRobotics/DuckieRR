#!/usr/bin/env python
from dagu_car.dagu_wheels_driver import DaguWheelsDriver
from rr_utils import (RRNodeInterface, LaunchRRNode, FormatRobdefString)
import sys
import argparse
import yaml
import numpy as np
import RobotRaconteur as RR

RRN = RR.RobotRaconteurNode.s

class DriveNode(RRNodeInterface):
    def __init__(self):
        self.node_name = "drive"
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
        self.driver.setWheelsSpeed(left=u_l_limited, right=u_r_limited)

    def toggleEStop(self):
        self._estop = not self._estop
        if self._estop:
            self.driver.setWheelsSpeed(left=0.0, right=0.0)
            self.log("Emergency Stop Activated")
        else:
            self.log("Emergency Stop Released")

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

        
    def onShutdown(self):
        self.driver.setWheelsSpeed(left=0.0, right=0.0)
        self.log("Shutting Down")


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
    
    launch_file = """\
node_name: Duckiebot.Drive

objects:
    - name: Drive
      robdef: ${DRIVE_ROBDEF}
      class: DriveNode.DriveNode

tcp_port: %d
    """%(args.port)
    launch_config = yaml.load(launch_file)
    
    LaunchRRNode(**launch_config)
    
