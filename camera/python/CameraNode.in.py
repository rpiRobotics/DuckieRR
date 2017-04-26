#!/usr/bin/env python
import sys
import argparse
import io
import numpy as np
import picamera
import picamera.array
import thread
import time
import yaml
from rr_utils import (RRNodeInterface, LaunchRRNode)
import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s
RRN.UseNumPy = True

class CameraNode(RRNodeInterface):
    def __init__(self):
        self.node_name = "camera"

        self._capturing = False
        self.update_framerate = False
        self.update_format = False

        self.framerate_high = 30.0
        self.framerate_low = 15.0
        self._framerate = self.framerate_high

        self.res_w = int(640)
        self.res_h = int(480)
        self._resolution = [self.res_w, self.res_h]

        self.format_options = ['jpeg','rgb','bgr']
        self._format = 'jpeg'
        self.newformat = None

        self.camera = picamera.PiCamera()
        self.camera.resolution = (self.res_w, self.res_h)
        self.camera.framerate = self._framerate
        time.sleep(2)

        self.stream = io.BytesIO()
        self.frame = 0

        self.image = RRN.NewStructure("Duckiebot.Image")
        self.image.width = self.res_w
        self.image.height = self.res_h
        self.image.format = self._format
        self.image.header = RRN.NewStructure("Duckiebot.Header")
        self.image.header.seq = 0
        self.image.header.time = 0.0
        self.image.header.ctime = 0.0

        self._imagestream = None

        self.log("Initialized.")


    def toggleFramerate(self):
        if self._framerate != self.framerate_high:
            self._framerate = self.framerate_high
        else:
            self._framerate = self.framerate_low

        self.update_framerate = True

    def changeFormat(self, newformat):
        if newformat not in self.format_options:
            raise ValueError("Error: Valid formats are %r"%self.format_options)
        else:
            if self._capturing:
                self.newformat = newformat
                self.update_format = True
            else:
                self._format = newformat
                self.image.format = self._format


    def captureImage(self):
        if self._capturing:
            raise RuntimeError('Can not capture image while streaming')

        self.camera.capture(self.stream, format=self._format, use_video_port=True)
        self.stream.seek(0)

        data = self.stream.getvalue()
        self.image.data = bytearray(data)
        self.image.header.seq = 0
        self.image.header.time = time.time()
        self.image.header.ctime = time.clock()

        # clear the stream
        self.stream.seek(0)
        self.stream.truncate()

        return self.image

    def startCapturing(self):
        if not self._capturing:
            self.log('Starting Capture')
            self._capturing = True
            thread.start_new_thread(self._capture_threadfunc,())

    def stopCapturing(self):
        if self._capturing:
            self.log('Stopping Capture')
            self._capturing = False

    def _capture_threadfunc(self):
        while self._capturing:
            gen = self._grabAndPublish(self.stream)
            try:
                start = time.time()
                self.camera.capture_sequence(gen,format=self._format, use_video_port=True)
            except StopIteration:
                pass

            finish = time.time()
            self.log('Captured %d %s frames at %0.2ffps'%(
                self.frame, self._format, self.frame/(finish-start)))
            self.frame = 0
            if self.update_framerate:
                self.log("Updating stream framerate...")
                self.camera.framerate = self._framerate
                self.update_framerate = False
                time.sleep(2)
                self.log("Stream framerate is now %0.2ffps"%(self._framerate))
            if self.update_format:
                self.log("Updating stream format...")
                self._format = self.newformat
                self.image.format = self._format
                self.update_format = False
                time.sleep(2)
                self.log("Stream format is now %s"%(self._format))

    def _grabAndPublish(self,stream):
        while (self._capturing and not self.update_framerate and not
        self.update_format):
            yield stream

            stream.seek(0)
            data = stream.getvalue()
            self.frame += 1

            #  fill the image data
            self.image.data = bytearray(data)
            self.image.header.seq = self.frame
            self.image.header.time = time.time()
            self.image.header.ctime = time.clock()

            #Publish using the pipe broadcaster
            self._imagestream_broadcaster.AsyncSendPacket(self.image,lambda:None)

            # clear the stream for the next iteration
            stream.seek(0)
            stream.truncate()
            time.sleep(0.001)

    @property
    def ImageStream(self):
        return self._imagestream
    @ImageStream.setter
    def ImageStream(self,value):
        self._imagestream = value
        self._imagestream_broadcaster = RR.PipeBroadcaster(value,1)

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
        self.log("Closing...")
        if self._capturing:
            self.stopCapturing()
        self._capturing = False
        time.sleep(2)
        self.camera.close()
        self.log("Shutdown.")

if __name__=='__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Initialize the Duckiebot')
    parser.add_argument('--port',type=int,default=0,
        help='TCP port to host service on' +\
         '(will auto-generate if not specified)')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])

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

