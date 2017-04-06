#!/usr/bin/env python
import sys
import argparse
import io
import numpy as np
from picamera import PiCamera
import threading, thread
import traceback
import time
import struct
import yaml
from rr_utils import (RRNodeInterface, LaunchRRNode, FormatRobdefString)
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

class CameraNode(RRNodeInterface):
    def __init__(self):
        self.node_name = "camera"

        self.framerate_high = 30.0
        self.framerate_low = 15.0
        self._framerate = self.framerate_high
        self.update_framerate = False

        self.res_w = int(640)
        self.res_h = int(480)
        self._resolution = [self.res_w, self.res_h]
        
        self.format_options = ('jpeg', 'rgb', 'bgr')
        self._format = 'rgb'

        
        self.camera = PiCamera()
        self.camera.framerate = self._framerate
        self.camera.resolution = (self.res_w, self.res_h)
        self.stream = io.BytesIO()

        self.is_shutdown = False
        
        self._lock = threading.RLock()

        self._image = RRN.NewStructure("Duckiebot.Image")
        self._image.width = self.res_w
        self._image.height = self.res_h
        self._image.format = self._format

        self._imagestream = None
        self._imagestream_endpoints = dict()
        self._imagestream_endpoints_lock = threading.RLock()
        self._capturing = False
        

    def toggleFramerate(self):
        if self._framerate != self.framerate_high:
            self._framerate = self.framerate_high
        else:
            self._framerate = self.framerate_low
        
        self.update_framerate = True

    def changeFormat(self, newformat):
        if newformat not in self.format_options:
            raise ValueError("Error: Valid formats are 'jpeg', 'rgb', 'bgr'")
        else:
            self._format = newformat
            self._image.format = newformat

    def captureImage(self):
        with self._lock:
            self.camera.capture(self.stream, format=self._format, use_video_port=True)
            self.stream.seek(0)
            
            data = self.stream.getvalue()
            self._image.data = bytearray(data) # must cast as byte array for RR

            # clear stream
            self.stream.seek(0)
            self.stream.truncate()

            return self._image


    def startCapturing(self):
        if (self._capturing):
            raise Exception('Already Capturing')
        self.log("Starting Capture")
        self._capturing = True
        t = threading.Thread(target=self._capture_threadfunc)
        t.start()

    def stopCapturing(self):
        if (not self._capturing):
            raise Exception('Not Capturing')
        self.log("Stopping Capture")
        self._capturing = False

    def _capture_threadfunc(self):
        while self._capturing and not self.is_shutdown:
            gen = self._grabAndPublish(self.stream)
            try:
                self.camera.capture_sequence(gen, format=self._format, use_video_port=True)
            except StopIteration:
                pass
            self.log("Updating framerate")
            self.camera.framerate = self._framerate
            self.update_framerate = False
        self.log("Capture Ended.")

    def _grabAndPublish(self,stream):
        while self._capturing and not self.update_framerate and not self.is_shutdown:
            yield stream

            with self._lock:
                stream.seek(0)
                data = stream.getvalue()
                self._image.data = bytearray(data)
            
            # send the new frame to the broadcaster using AsyncSendPacket
            # and a blank handler. We don't really care when the send finishes
            # since we are using the "backlog" flow control in the broadcaster
            self._imagestream_broadcaster.AsyncSendPacket(self._image,lambda: None)
            # clear stream
            stream.seek(0)
            stream.truncate()

            time.sleep(0.0001)

    @property
    def ImageStream(self):          
        return self._imagestream
    @ImageStream.setter
    def ImageStream(self,value):
        self._imagestream = value
        # Create a PipeBroadcaster and allow a backlog of 3 images.
        self._imagestream_broadcaster=RR.PipeBroadcaster(value,3)
    
    @property
    def framerate(self):
        return self._framerate

    @property 
    def resolution(self):
        return self._resolution

    @property
    def format(self):
        return self._format

    @property
    def capturing(self):
        if self._capturing:
            return 1
        else:
            return 0

    def onShutdown(self):
        self.log("Closing Camera.")
        self.camera.close()
        self.is_shutdown = True
        self.log("Shutdown.")

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
    
    launch_file="""\
node_name: Duckiebot.Camera

objects:
    - name: Duckiebot
      robdef: ${DUCKIEBOT_ROBDEF}
    - name: Camera
      robdef: ${CAMERA_ROBDEF}
      class: CameraNode.CameraNode

tcp_port: %d
    """%(args.port)
    
    launch_config = yaml.load(launch_file)

    LaunchRRNode(**launch_config)
