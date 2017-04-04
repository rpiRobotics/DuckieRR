#!/usr/bin/env python
import sys
import argparse
import numpy as np
import time
import threading, thread
import pygame
import yaml
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
from rr_utils import *
from duckie_utils.configurable import Configurable


class JoyMapperNode(RRNodeInterface,Configurable):
    def __init__(self, configuration):
        self.node_name = "joyMapper"
        private_param_names = [
            'speed_gain', 
            'steer_gain', 
            'car_like', 
            'steer_angle_gain',
            'simulated_vehicle_length',
            'mapping']
        Configurable.__init__(self, private_param_names, configuration,makePrivate=True)
        
        self.buttons = ['A','B','X','Y',
            'LB','RB','BACK','START','LOGITECH',
            'LAXISBUTTON', 'RAXISBUTTON']
        self.motion_axes = ['LXAXIS','LYAXIS','RXAXIS','RYAXIS']
        self.trigger_axes = ['LT','RT']
        self.dpad = ['DPADX','DPADY']

        if not self.validate_map(self._mapping):
            raise RuntimeError("[%s] Invalid Key Mapping from Config"%self.node_name)
        
        self.joy = self.FindAndConnect("Joy.Joystick")
        self.drive = self.FindAndConnect("Duckiebot.Drive.Drive",required=False)
        
        self.joy.joyChange += self.cbJoy
        self.newJoyData = False

        self._running = True
        self._t_joyworker = threading.Thread(target=self._joyworkerthread)
        self._t_joyworker.daemon = True
        self._t_joyworker.start()

    def onShutdown(self):
        self._running = False
        self._t_joyworker.join()
        self.drive.carCmd(0,0)
        self.log("Shutting Down JoyMapper")
        

    @property
    def speed_gain(self):
        return self._speed_gain
    @speed_gain.setter
    def speed_gain(self,value):
        self._speed_gain = value

    @property
    def steer_gain(self):
        return self._steer_gain
    @steer_gain.setter
    def steer_gain(self,value):
        self._steer_gain = value

    @property
    def car_like(self):
        return self._car_like

    @car_like.setter
    def car_like(self,value):
        self._car_like = value

    @property
    def steer_angle_gain(self):
        return self._steer_angle_gain
    @steer_angle_gain.setter
    def steer_angle_gain(self,value):
        self._steer_angle_gain = value

    @property
    def simulated_vehicle_length(self):
        return self._simulated_vehicle_length
    @simulated_vehicle_length.setter
    def simulated_vehicle_length(self,value):
        self._simulated_vehicle_length = value

    @property 
    def mapping(self):
        return self._mapping
    @mapping.setter
    def mapping(self,newmap):
        if self.validate_map(newmap):
            self._mapping = newmap

    def validate_map(self,newmap):
        ret = True
        valid_mappings = {'v':self.motion_axes, 
                          'w':self.motion_axes,
                          'estop':self.buttons,
                          'carlike':self.buttons}
        for key,value in newmap.iteritems():
            if key in valid_mappings:
                if value not in valid_mappings[key]:
                    self.log("'%s' is not an allowed mapping for '%s'"%(value,key))
                    ret = False        
            else:
                self.log("'%s' is not currently a valid function to map"%key)
                ret = False
        return ret

    def cbJoy(self):
        self.newJoyData = True

    def _joyworkerthread(self):
        while self._running:
            if self.newJoyData:
                self.newJoyData = False
                self.publishControl()
                self.processButtons()
            time.sleep(0.01)

    def publishControl(self):
        v = -self.joy.axes[self._mapping['v']]*self._speed_gain #(-) so that up is +'ve
        if self.car_like:
            # Implement Bicycle Kinematics - Nonholonomic Kinematics
            # see https://inst.eecs.berkeley.edu/~ee192/sp13/pdf/steer-control.pdf
            steering_angle = -self.joy.axes[self._mapping['w']]*self._steer_angle_gain
            w = v / self._simulated_vehicle_length * np.tan(steering_angle)
        else:
            # Holonomic Kinematics for Normal Driving
            w = -self.joy.axes[self._mapping['w']] * self._steer_gain
            #print "(%0.3f, %0.3f)"%(v,w)
            self.drive.carCmd(v,w)

    def processButtons(self):
        if self.joy.buttons[self._mapping['estop']] == 1:
            self.drive.toggleEStop()
        if self.joy.buttons[self._mapping['carlike']] == 1:
            self._car_like ^= 1 # (XOR)
            self.log('car_like: %d')%self._car_like

if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Initialize the joystick')
    parser.add_argument('--port',type=int,default=0,
        help='TCP port to host service on' +\
        '(will auto-generate if not specified)')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])

    launch_file = """\
node_name: Duckiebot.JoyMapper

objects:
    - name: JoyMapper
      robdef: ${JOYMAPPER_ROBDEF}
      class: JoyMapperNode.JoyMapperNode
      configuration: ${DEFAULT_MAPPER_PARAMS}

tcp_port: %d
    """%(args.port)
    
    launch_config = yaml.load(launch_file)
    LaunchRRNode(**launch_config)
    

    
