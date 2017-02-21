#!/usr/bin/env python
from duckie_utils.configurable import Configurable
from duckie_utils.instantiate_utils import instantiate
from duckie_utils.image import DuckieImageToBGRMat
from duckie_utils.stats import Stats
from rr_utils import (RRNodeInterface, LaunchRRNode, FormatRobdefString)
import cv2
import numpy as np
import threading
import time
import yaml
import sys, argparse

import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

class LineDetectorNode(Configurable,RRNodeInterface):
    """Line Detector Node will return detected lines."""
    def __init__(self, configuration):
        self.node_name = "lineDetector"

        # initialize configurable
        param_names = [
            'img_size',
            'top_cutoff',
            'detector'
            ]
        Configurable.__init__(self,param_names,configuration)

        self._segments = []
        self.RRConsts = RRN.GetConstants("Duckiebot")
        
        # Thread lock
        self.thread_lock = threading.Lock()

        self.active = True

        self.stats = Stats()

        # only print every 10 cycles
        self.intermittent_interval = 100
        self.intermittent_counter = 0

        # extract the detector params
        c = self.detector
        assert isinstance(c,list) and len(c) == 2, c

        self.log("new detector config: %s"%(str(c)) )

        # instantiate it
        #   c[0] is the detector type -- e.g. line_detector.LineDetectorHSV
        #   c[1] is any inpit args (the configuration dictionary)
        self.detector = instantiate(c[0],c[1])
        
        # Find and connect to the image service
        self.duckie_cam = self.FindAndConnect("Duckiebot.Camera.Camera")
        
        # connect to the pipe
        self.imstream = self.duckie_cam.ImageStream.Connect(-1) # connect to the pipe
        self.imstream.PacketReceivedEvent+=self._cbImage

        try:
            self.duckie_cam.startCapturing()
        except: pass

        self.newSegments = RR.EventHook()
    
    @property
    def segments(self):
        return self._segments

    def intermittent_log_now(self):
        return self.intermittent_counter % self.intermittent_interval == 1

    def intermittent_log(self,s):
        if not self.intermittent_log_now():
            return
        msg = "%3d:%s"%(self.intermittent_counter, s)
        self.log(msg)

    def _cbImage(self, pipe_ep):
        while (pipe_ep.Available > 0):
            self.stats.received()
            image=pipe_ep.ReceivePacket()

        if not self.active:
            return

        #start a daemon thread to process the image
        thread = threading.Thread(target=self._processImage, args=(image,))
        thread.setDaemon(True)
        thread.start()
        # this returns right away...

    def _processImage(self,image):
        if not self.thread_lock.acquire(False): # False indicates non-blocking
            self.stats.skipped()
            # return immediately if the thread is locked
            return

        try:
            self.__processImage(image)
        finally:
            # release the thread lock
            self.thread_lock.release()

    def __processImage(self,image):
        self.stats.processed()

        if self.intermittent_log_now():
            self.intermittent_log(self.stats.info())
            self.stats.reset()

        self.intermittent_counter += 1

        # extract the image data
        try:
            image_cv = DuckieImageToBGRMat(image)
        except ValueError as e:
            self.log("Could not decode image: %s"%(e))
            return

        # resize and crop image
        h_original, w_original = image_cv.shape[0:2]

        if self.img_size[0] != h_original or self.img_size[1] != w_original:
            image_cv = cv2.resize(image_cv, (self.img_size[1], self.img_size[0]),
                interpolation=cv2.INTER_NEAREST)
            image_cv = image_cv[self.top_cutoff:,:,:]

        # apply color correction ... 
        # ADD IN LATER IF NEEDED

        # set the image to be detected
        self.detector.setImage(image_cv)

        # Detect lines and normals
        white = self.detector.detectLines('white')
        yellow = self.detector.detectLines('yellow')
        red = self.detector.detectLines('red')

        # Reset the segments list
        self._segments = []

        # convert to normalized pixel coordinates, and add segments to segment list
        arr_cutoff = np.array((0, self.top_cutoff, 0, self.top_cutoff))
        arr_ratio = np.array((1./self.img_size[1], 1./self.img_size[0], 1./self.img_size[1], 1./self.img_size[0] ))
        
        if len(white.lines) > 0:
            lines_normalized_white = ((white.lines + arr_cutoff) * arr_ratio)
            self._segments.extend(self.toSegment(lines_normalized_white, white.normals, self.RRConsts.WHITE))
        
        if len(yellow.lines) > 0:
            lines_normalized_yellow = ((yellow.lines + arr_cutoff) * arr_ratio)
            self._segments.extend(self.toSegment(lines_normalized_yellow, yellow.normals, self.RRConsts.YELLOW))
        
        if len(red.lines) > 0:
            lines_normalized_red = ((red.lines + arr_cutoff) * arr_ratio)
            self._segments.extend(self.toSegment(lines_normalized_red, red.normals, self.RRConsts.RED))

        self.newSegments.fire(self._segments)
        self.intermittent_log('# segments: white %3d yellow %3d red %3d' % (len(white.lines),
                len(yellow.lines), len(red.lines)))


    def toSegment(self, lines, normals, color):
        segmentList = []
        for x1,y1,x2,y2,norm_x,norm_y in np.hstack((lines,normals)):
            segment = RRN.NewStructure("Duckiebot.Segment")
            segment.pixels_normalized = []
            segment.color = color

            vec = RRN.NewStructure("Duckiebot.Vector2D")
            vec.x = x1
            vec.y = y1
            segment.pixels_normalized.append(vec)
            
            vec.x = x2
            vec.y = y2
            segment.pixels_normalized.append(vec)
            
            vec.x = norm_x
            vec.y = norm_y
            segment.normal = vec

            segmentList.append(segment)
        return segmentList

    def onShutdown(self):
        self.imstream.Close()
        self.log("Shutdown.")


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Initialize the line detector')
    parser.add_argument('--config', type=open,
        help='A config file for the line detector (Otherwise use Default)')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])

    config_file = args.config
    if config_file is None:
        config_file = '${DEFAULT_LD_PARAMS}'

    launch_file = """\
node_name: Duckiebot.LineDetector

objects:
    - name: Duckiebot
      robdef: ${DUCKIEBOT_ROBDEF}

    - name: LineDetector
      robdef: ${LINEDETECTOR_ROBDEF}
      class: LineDetectorNode.LineDetectorNode
      configuration: %s 

    """%(config_file)
    
    launch_config = yaml.load(launch_file)
    LaunchRRNode(**launch_config)