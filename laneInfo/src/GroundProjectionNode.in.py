#!/usr/bin/env python
from duckie_utils.configurable import Configurable
from duckie_utils.instantiate_utils import instantiate
from duckie_utils.image import DuckieImageToBGRMat
from duckie_utils.stats import Stats
from rr_utils import *
import cv2
import numpy as np
import threading
import time
import yaml
import sys, argparse

import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

class GroundProjectionNode(Configurable,RRNodeInterface):
    """Line Detector Node will return detected lines."""
    def __init__(self, configuration):
        self.node_name = "groundProjection"

        # initialize configurable
        param_names = [
            'board_w',
            'board_h',
            'square_size',
            'x_offset',
            'y_offset'
            ]
        Configurable.__init__(self,param_names,configuration)

        self._segments = []
        self.Consts = RRN.GetConstants("Duckiebot")
        
        # Thread lock
        self.thread_lock = threading.Lock()

        self.active = True

        self.stats = Stats()

        # only print every 10 cycles
        self.intermittent_interval = 100
        self.intermittent_counter = 0
        
        # Find the line detector service
        self.ld = self.FindAndConnect("Duckiebot.LineDetector.LineDetector")
        
        # and connect to the new segments event
        self.ld.newSegments += self._cbLineSeg
        self.newSegments = RR.EventHook()

        # read the homography matrix from file
        h_file = "${DEFAULT_CAMEXT}"
        with open(h_file,'r') as f:
            h_data = yaml.load(f.read());
            self.H = h_data['homography'].reshape((3,3))
            self.Hinv = np.linalg.inv(self.H)

        self.rectified_input = False

        cam_file = "${DEFAULT_CAMINT}"
        with open(cam_file, 'r') as f:
            cam_data = yaml.load(f.read());
            self.cam_info = RRN.NewStructure("Duckiebot.CameraInfo")
            self.cam_info.width = cam_data['image_width']
            self.cam_info.height = cam_data['image_height']
            d = cam_data['distortion_coefficients']
            self.cam_info.D = d['data'].reshape((d['rows'],d['cols']))
            k = cam_data['camera_matrix']
            self.cam_info.K = k['data'].reshape((k['rows'],k['cols']))
            r = cam_data['rectification_matrix']
            self.cam_info.R = r['data'].reshape((r['rows'],r['cols']))
            p = cam_data['projection_matrix']
            self.cam_info.P = p['data'].reshape((p['rows'],p['cols']))


        '''
        # CAMERA CURRENTLY NOT NEEDED
        # Find the image service and connect to the pipe
        self.duckie_cam = self.FindAndConnect("Duckiebot.Camera")
        
        self.imstream = self.duckie_cam.ImageStream.Connect(-1) # connect to the pipe
        self.imstream.PacketReceivedEvent+=self._cbImage

        try:
            self.duckie_cam.startCapturing()
        except: pass
        '''
    ################################
    # SERVICE DEFINITION FUNCTIONS #
    ################################
    @property
    def segments(self):
        return self._segments


    def estimate_homography(self, image):
        '''
        Estimating the homography using the camera and a calib grid...
        '''
        raise NotImplementedError("estimate_homography is not currently implemented")

    def ground_coordinate(self, normalized_uv):
        '''
        estimate the ground coordinate from a pixel and its normalized form
        '''
        gp = self.image2ground(normalized_uv)
        return gp
    
    def image_coordinate(self, gp):
        '''
        Estimate the image coordinate that corresponds to a ground point
        '''
        pixel = self.ground2image(gp)
        normalized_uv  = pixel2vector(pixel)
        return normalized_uv

    #####################
    # PRIVATE FUNCTIONS #
    #####################

    def intermittent_log_now(self):
        return self.intermittent_counter % self.intermittent_interval == 1

    def intermittent_log(self,s):
        if not self.intermittent_log_now():
            return
        msg = "%3d:%s"%(self.intermittent_counter, s)
        self.log(msg)

    def _cbLineSeg(self, linesegs):
        self.stats.received()

        if not self.active:
            return

        #start a daemon thread to process the image
        thread = threading.Thread(target=self._processSegments, args=(linesegs,))
        thread.setDaemon(True)
        thread.start()
        # this returns right away...

    def _processSegments(self,linesegs):
        if not self.thread_lock.acquire(False): # False indicates non-blocking
            self.stats.skipped()
            # return immediately if the thread is locked
            return

        try:
            self.__processSegments(linesegs)
        finally:
            # release the thread lock
            self.thread_lock.release()

    def __processSegments(self,linesegs):
        self.stats.processed()

        if self.intermittent_log_now():
            self.intermittent_log(self.stats.info())
            self.stats.reset()

        self.intermittent_counter += 1

        self._segments = linesegs
        for seg in self._segments:
            seg.points = []

            assert(len(seg.pixels_normalized) == 2)
            
            point1 = self.image2ground(seg.pixels_normalized[0])
            point2 = self.image2ground(seg.pixels_normalized[1])
            seg.points.extend([point1, point2])

        self.newSegments.fire(self._segments)

    def image2ground(self, pix):
        try: 
            u = pix.u
            v = pix.v
        except AttributeError:
            # we must have been passed a vector
            pix = vector2pixel(pix)
            u = pix.u
            v = pix.v

        pt_img = np.array([u,v,1.0])

        if not self.rectified_input:
            pt_undistorted = self.rectifyCVPoint(pt_img[0:2])
            pt_img[0:2] = pt_undistorted[0:2]

        pt_gnd = np.dot(self.H,pt_img);
        
        point = RRN.NewStructure("Duckiebot.Point")
        point.x = pt_gnd[0]/pt_gnd[2]
        point.y = pt_gnd[1]/pt_gnd[2]
        point.z = 0.0

        return point

    def ground2image(self, point):
        pt_gnd = np.array([point.x, point.y, 1.0])

        pt_img = np.dot(self.Hinv, pt_gnd)
        pt_img[0:2] /= pt_img[2]
        
        if not self.rectified_input:
            pt_distorted = self.unrectifyCVPoint(pt_gnd[0:2])
            pt_img[0:2] = pt_distorted[0:2]

        pixel = RRN.NewStructure("Duckiebot.Pixel")
        pixel.u = int(pt_img[0])
        pixel.v = int(py_img[1])
        return pixel

    def rectifyCVPoint(self,raw2d):
        src_pt = raw2d.astype('double')
        rectified = cv2.undistortPoints(src_pt, self.cam_info.K, self.cam_info.D, self.cam_info.R, self.cam_info.P)
        return rectified

    def unrectifyCVPoint(rect2d):
        uv_rect = rect2d.astype('double')
        # extract some paremeters from the camera matrices
        R = self.cam_info.R    
        D = self.cam_info.D
        K = self.cam_info.K
        P = self.cam_info.P

        cx = P[0,2]; cy = P[1,2]
        fx = P[0,0]; fy = P[1,1]
        Tx = P[0,3]; Ty = P[1,3]

        # Formula from docs for cv2.initUndistortRectifyMap
        # x <- (u - c'x) / f'x
        # y <- (v - c'y) / f'y
        # c'x, f'x, etc. (primed) come from "new camera matrix" P[0:3, 0:3]
        x = (uv_rect[0] - cx - Tx)/ fx;
        y = (uv_rect[1] - cy - Ty)/ fy;
        # [X Y W]^T <- R^-1 * [x y 1]^T
        X,Y,W = np.dot(np.linalg.inv(R), np.array([x,y,1.0]))
        # x' <- X/W, y' <- Y/W
        xp = X/W
        yp = Y/W
        # x'' <- x'(1+k1*r^2+k2*r^4+k3*r^6) + 2p1*x'*y' + p2(r^2+2x'^2)
        # y'' <-y'(1+k1*r^2+k2*r^4+k3*r^6) + p1(r^2+2y'^2) + 2p2*x'*y'  
        # where r^2 = x'^2 + y'^2
        r2 = xp*xp + yp*yp
        r4 = r2*r2
        r6 = r4*r2
        a1 = 2*xp*yp
        k1 = D[0,0]; k2 = D[0,1]; p1 = D[0,2]; p2 = D[0,3]; k3 = D[0,4]
        barrel_correction = 1+ k1*r2 + k2*r4 + k3*r6
        if (D.shape[1] == 8):
            barrel_correction /= (1.0 + D[0,5]*r2 + D[0,6]*r4 + D[0,7]*r6)
        xpp = xp*barrel_correction + p1*a1 + p2*(r2+2*(xp*xp))
        ypp = yp*barrel_correction + p1*(r2+2*(yp*yp)) + p2*a1
        # map_x(u,v) <- x''fx + cx
        # map_y(u,v) <- y''fy + cy
        # cx,fx, etc. come from original camera matrix K
        map_x = xpp*K[0,0] + K[0,2]
        map_y = ypp*K[1,1] + K[1,2]
        unrectified = np.array([map_x, map_y])
        return unrectified

    def vector2pixel(self, vec2d):
        w = float(self.cam_info.width)
        h = float(self.cam_info.height)
        pixel = RRN.NewStructure("Duckiebot.Pixel")
        pixel.u = int(w*vec2d.x)
        pixel.v = int(h*vec2d.y)

        # boundary check
        pixel.u = np.clip(pixel.u, 0, w-1)
        pixel.v = np.clip(pixel.v, 0, h-1)

        return pixel

    def pixel2vector(self, pixel):
        w = float(self.cam_info.width)
        h = float(self.cam_info.height)

        vector = RRN.NewStructure("Duckiebot.Vector")
        vector.x = float(pixel.u)/w
        vector.y = float(pixel.v)/h
        return vector

    def onShutdown(self):
        self.log("Shutdown.")


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Initialize the ground projection')
    parser.add_argument('--config', type=open,
        help='A config file for the ground projection (Otherwise use Default)')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])

    config_file = args.config
    if config_file is None:
        config_file = '${DEFAULT_GP_PARAMS}'

    launch_file = """\
node_name: Duckiebot.LineDetector

objects:
    - name: Duckiebot
      robdef: ${DUCKIEBOT_ROBDEF}
    - name: LineDetector
      robdef: ${GROUNDPROJECTION_ROBDEF}
      class: LineDetectorNode.LineDetectorNode
      configuration: %s 

    """%(config_file)
    
    launch_config = yaml.load(launch_file)
    LaunchRRNode(**launch_config)