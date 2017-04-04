#!/usr/bin/env python
import sys
import os
import argparse
import numpy as np
import time
import threading, thread
import pygame
import yaml
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
from rr_utils import *


# Axis 0 - Laxis - Left(-1)/Right(1)
# Axis 1 - Laxis - Up(-1)/Down(1)
# Axis 2 - LT - Up(-1)/Down(1)
# Axis 3 - Raxis - Left(-1)/Right(1)
# Axis 4 - Raxis - Up(-1)/Down(1)
# Axis 5 - RT - Up(-1)/Down(1)
# Axis 6 - DPADX - Left(-1)/Right(1)
# Axis 7 - DPADY - Up(1)/Down(-1)
# Button 0 - A
# Button 1 - B
# Button 2 - X
# Button 3 - Y
# Button 4 - LB
# Button 5 - RB
# Button 6 - Back
# Button 7 - Start
# Button 8 - Logitech
# Button 9 - Laxis press
# Button 10- Raxis press
# (Hat 0) - DPAD(X,Y) -- (-1=Left/Down, +1=Right/Up) -- only on some machines
# Don't SWITCH THE D/X SWITCH!
# mode and vibration not mapped

class JoyNode(RRNodeInterface):
    def __init__(self):
        self.node_name = "joy"
        
        self.verbose = False
        os.environ["SDL_VIDEODRIVER"] = "dummy"
	pygame.init()
        #pygame.display.init()
        pygame.joystick.init()
        if not pygame.joystick.get_count():
            raise RuntimeError('No joystick detected')
        
        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()

        self._numbuttons = self.joy.get_numbuttons()
        self._numaxes = self.joy.get_numaxes()
	num_hats = self.joy.get_numhats()
	if num_hats != 0:
	    self._numaxes += 2*num_hats
	    self.dpad = True

        self.buttonmap = {0:'A',1:'B',2:'X',3:'Y',
        4:'LB',5:'RB',6:'BACK',7:'START',8:'LOGITECH',
        9:'LAXISBUTTON', 10:'RAXISBUTTON'}
        self.axesmap={0:'LXAXIS',1:'LYAXIS',2:'LT',3:'RXAXIS',4:'RYAXIS',5:'RT',6:'DPADX',7:'DPADY'}

        self._buttons = dict(zip(self.buttonmap.values(),[0]*self._numbuttons))
        self._axes = dict(zip(self.axesmap.values(),[0]*self._numaxes))



        self.buttonDown = RR.EventHook()
        self.buttonUp = RR.EventHook()
        self.axisMotion = RR.EventHook()
        self.joyChange = RR.EventHook()

        # start processing threads
        self._running = True
        self._t_joy = threading.Thread(target=self._joystickthread)
        self._t_joy.daemon = True
        self._t_joy.start()

        self.log("Joystick Node Started")

    def onShutdown(self):
        self.log("Shutting Down Joystick")
        self._running = False
        self._t_joy.join()

        pygame.joystick.quit()
        pygame.quit()

    @property
    def buttons(self):
        return self._buttons
    @property
    def axes(self):
        return self._axes

    def _joystickthread(self):
        while self._running:
            #tic = time.time()
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    but = event.__dict__['button']
                    self._buttons[self.buttonmap[but]] = 1
                    self.buttonDown.fire(self.buttonmap[but])
                    if self.verbose:
                        print "Button %s Down"%(self.buttonmap[but])
                
                if event.type == pygame.JOYBUTTONUP:
                    but = event.__dict__['button']
                    self._buttons[self.buttonmap[but]] = 0
                    self.buttonUp.fire(self.buttonmap[but])
                    if self.verbose:
                        print "Button %s Up"%(self.buttonmap[but])
                
                if event.type == pygame.JOYAXISMOTION:
                    ax = event.__dict__['axis']
                    val = event.__dict__['value']
                    self._axes[self.axesmap[ax]] = val
                    self.axisMotion.fire(self.axesmap[ax], val)
                    if self.verbose:
                        print "Axis %s value: %0.3f"%(self.axesmap[ax],val)

                if event.type == pygame.JOYHATMOTION:
                    val = list(event.__dict__['value'])
                    self._axes['DPADX'] = val[0]
                    self._axes['DPADY'] = val[1]
                    self.axisMotion.fire('DPADX',val[0])
		    self.axisMotion.fire('DPADY',val[1])
                    if self.verbose:
                        print "DPad value: %r"%val

                self.joyChange.fire()

            time.sleep(0.001)    

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
node_name: Joy

objects:
    - name: Joystick
      robdef: ${JOYSTICK_ROBDEF}
      class: JoyNode.JoyNode

tcp_port: %d
    """%(args.port)
    
    launch_config = yaml.load(launch_file)
    LaunchRRNode(**launch_config)
    

    
