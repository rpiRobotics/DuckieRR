#!/usr/bin/env python
import sys
import argparse
import io
import socket
import numpy as np
from picamera import PiCamera
import threading, thread
import traceback
import time

import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

camera_servicedef="""
#Service to provide interface to Duckiebot
service Duckiebot_Interface

option version 0.8

struct DuckieImage
    field string format
    field int32 width
    field int32 height
    field uint8[] data
end struct

object Duckiebot_Camera

property double framerate
property int32[] resolution
property string format
property uint8 capturing

function void startCapturing()
function void stopCapturing()
function DuckieImage captureImage()

function void toggleFramerate()
function void changeFormat(string format)

pipe DuckieImage ImageStream

end object

"""

class Camera_impl(object):
    def __init__(self):
        self.nodeName = "camera"

        self.framerate_high = 30.0
        self.framerate_low = 15.0
        self._framerate = self.framerate_high
        self.update_framerate = False

        self.res_w = int(640)
        self.res_h = int(480)
        self._resolution = [self.res_w, self.res_h]
        
        self.format_options = ('jpeg', 'rgb', 'bgr')
        self._format = 'jpeg'

        
        self.camera = PiCamera()
        self.camera.framerate = self._framerate
        self.camera.resolution = (self.res_w, self.res_h)
        self.stream = io.BytesIO()

        self.is_shutdown = False
        
        self._lock = threading.RLock()

        self._image = RRN.NewStructure("Duckiebot_Interface.DuckieImage")
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
            
            self._image.data = self.stream.getvalue()

            # clear stream
            self.stream.seek(0)
            self.stream.truncate()

            return self._image


    def startCapturing(self):
        if (self._capturing):
            raise Exception('Already Capturing')
        print "[%s] Starting Capture"%(self.nodeName)
        self._capturing = True
        t = threading.Thread(target=self._capture_threadfunc)
        t.start()

    def stopCapturing(self):
        if (not self._capturing):
            raise Exception('Not Capturing')
        print "[%s] Stopping Capture"%(self.nodeName)
        self._capturing = False

    def _capture_threadfunc(self):
        while self._capturing and not self.is_shutdown:
            gen = self._grabAndPublish(self.stream)
            try:
                self.camera.capture_sequence(gen, format=self._format, use_video_port=True)
            except StopIteration:
                pass
            print "[%s] Updating framerate"%(self.nodeName)
            self.camera.framerate = self._framerate
            self.update_framerate = False
        print "[%s] Capture Ended."%(self.nodeName)

    def _grabAndPublish(self,stream):
        while self._capturing and not self.update_framerate and not self.is_shutdown:
            yield stream

            with self._lock:
                stream.seek(0)
                self._image.data = stream.getvalue()
            with self._imagestream_endpoints_lock:
                # send to pipe endpoints
                for ep in self._imagestream_endpoints:
                    try:
                        # try to send the frame to the connected endpoint
                        pipe_ep = self._imagestream_endpoints[ep]
                        pipe_ep.SendPacket(self._image)
                    except:
                        # if there is an error, assume endpoint has closed
                        self._ImageStream_pipeclosed(pipe_ep)
            
            # clear stream
            stream.seek(0)
            stream.truncate()

            time.sleep(0.001)

    @property
    def ImageStream(self):          
        return self._imagestream
    @ImageStream.setter
    def ImageStream(self,value):
        self._imagestream = value
        # Set the PipeConnectCallback to _ImageStream_pipeconnect
        value.PipeConnectCallback=self._ImageStream_pipeconnect

    def _ImageStream_pipeconnect(self, pipe_ep):
        "Called when the PipeEndpoint Connects. pipe_ep is the endpoint"
        # lock the _imagestream_endpoints dictionary, and place pipe_ep in it
        with self._imagestream_endpoints_lock:
            # Add pipe_ep to the dictionary by endpoint and index
            self._imagestream_endpoints[(pipe_ep.Endpoint, pipe_ep.Index)]=pipe_ep
            # set the function to call when the pip endpoint is closed
            pipe_ep.PipeEndpointClosedCallback = self._ImageStream_pipeclosed

    def _ImageStream_pipeclosed(self,pipe_ep):
        with self._imagestream_endpoints_lock:
            try:
                del(self._imagestream_endpoints[(pipe_ep.Endpoint, pipe_ep.Index)])
            except:
                traceback.print_exc()

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

    def on_shutdown(self):
        print "[%s] Closing Camera."%(self.nodeName)
        self.camera.close()
        self.is_shutdown = True
        print "[%s] Shutdown."%(self.nodeName)      

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
    RRN.UseNumPy = True

    # Create Local Transport, start server as name, and register it
    t1 = RR.LocalTransport()
    t1.StartServerAsNodeName("DuckiebotServer.Camera")
    RRN.RegisterTransport(t1)

    # Create TCP Transport, register it, and start the server  
    t2 = RR.TcpTransport()
    t2.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
            RR.IPNodeDiscoveryFlags_LINK_LOCAL |
            RR.IPNodeDiscoveryFlags_SITE_LOCAL)

    RRN.RegisterTransport(t2)

    port = args.port
    t2.StartServer(port)
    if (port == 0):
        port = t2.GetListenPort()


    # Register the service def
    RRN.RegisterServiceType(camera_servicedef)

    # Initialize the object
    camera_obj = Camera_impl()
    
    # Register the service
    RRN.RegisterService("Duckiebot_Camera","Duckiebot_Interface.Duckiebot_Camera", camera_obj)

    print "Service started, connect via one of the following:"
    print "rr+local:///?nodename=DuckiebotServer.Camera&service=Duckiebot_Camera"
    print "rr+tcp://localhost:%s/?service=Duckiebot_Camera"%(port)
    print "rr+tcp://localhost:%s/?nodename=DuckiebotServer.Camera&service=Duckiebot_Camera"%(port)
    print "rr+tcp://%s.local:%s/?nodename=DuckiebotServer.Camera&service=Duckiebot_Camera"%(socket.gethostname(), port)
    print "rr+tcp://%s:%s/?nodename=DuckiebotServer.Camera&service=Duckiebot_Camera"%(socket.gethostbyname(socket.gethostname()), port)
    try:
        while True:
            pass
    except (KeyboardInterrupt,SystemExit):
        camera_obj.on_shutdown()
        
        # This must be here to prevent segfault
        RRN.Shutdown()
        sys.exit(0)
