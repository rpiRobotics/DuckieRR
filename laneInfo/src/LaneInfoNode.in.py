#!/usr/bin/env python
import numpy as np
import time
import yaml
import sys, argparse
from math import floor, atan2, pi, cos, sin, sqrt
from scipy.ndimage.filters import gaussian_filter
from scipy.stats import multivariate_normal, entropy

from duckie_utils.configurable import Configurable
from duckie_utils.stats import Stats
from duckie_utils.timekeeper import TimeKeeper
from duckie_utils.instantiate_utils import instantiate
from duckie_utils.image import DuckieImageToBGRMat

from rr_utils import (RRNodeInterface, LaunchRRNode)
from line_detector.line_detector_plot import *

import RobotRaconteur as RR
RRN = RR.RobotRaconteurNode.s

class LaneInfoNode(Configurable,RRNodeInterface):
    """Lane Info node will return position in lane"""
    def __init__(self, configuration):
        self.node_name = "laneInfo"

        self.initializeParams(configuration)

        self.active = True

        # This may need to change...
        self.RRConsts = RRN.GetConstants("Duckiebot")
        
        # LANE INFO SETUP
        self.d,self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,self.phi_min:self.phi_max:self.delta_phi]
        self.beliefRV=np.empty(self.d.shape)
        self.initializeBelief()
        self._lanePose = RRN.NewStructure("Duckiebot.LaneInfo.LanePose")
        self._lanePose.d=self.mean_0[0]
        self._lanePose.phi=self.mean_0[1]
        self._lanePose.sigma_d = self.sigma_d_0
        self._lanePose.sigma_phi = self.sigma_phi_0
        self._lanePose.in_lane = 0

        self.t_last_update = time.time()
        self.v_current = 0
        self.w_current = 0
        self.v_last = 0
        self.w_last = 0
        self.v_avg = 0
        self.w_avg = 0
        
        # LINE DETECTION SETUP
        self._verbose = False
        self._verboseImage = None
        self._pub_im = RRN.NewStructure("Duckiebot.Image")
        self._pub_im.height = self.img_size[0] - self.top_cutoff
        self._pub_im.width = self.img_size[1]
        self._pub_im.format = 'bgr'

        self.segment = RRN.NewStructure("Duckiebot.Segment")
        self.segment.pixels_normalized = [RRN.NewStructure("Duckiebot.Vector2D"),
            RRN.NewStructure("Duckiebot.Vector2D")]
        self.segment.normal = RRN.NewStructure("Duckiebot.Vector2D")
        self.segment.points = [RRN.NewStructure("Duckiebot.Point"), 
            RRN.NewStructure("Duckiebot.Point")]

        # read the homography matrix from file
        h_file = "${DEFAULT_CAMEXT}"
        with open(h_file,'r') as f:
            h_data = yaml.load(f.read());
            self.H = np.array( h_data['homography'] ).reshape((3,3))
            self.Hinv = np.linalg.inv(self.H)

        self.rectified_input = False

        cam_file = "${DEFAULT_CAMINT}"
        with open(cam_file, 'r') as f:
            cam_data = yaml.load(f.read());
            self.cam_info = RRN.NewStructure("Duckiebot.CameraInfo")
            self.cam_info.width = cam_data['image_width']
            self.cam_info.height = cam_data['image_height']
            d = cam_data['distortion_coefficients']
            self.cam_info.D =np.array( d['data'], dtype=np.float64 ).reshape((d['rows'],d['cols']))
            k = cam_data['camera_matrix']
            self.cam_info.K =np.array( k['data'], dtype=np.float64 ).reshape((k['rows'],k['cols']))
            r = cam_data['rectification_matrix']
            self.cam_info.R =np.array( r['data'], dtype=np.float64 ).reshape((r['rows'],r['cols']))
            p = cam_data['projection_matrix']
            self.cam_info.P =np.array( p['data'], dtype=np.float64 ).reshape((p['rows'],p['cols']))

        self.stats = Stats()
        self.tk = None # need to remember to set this when we want to start timing
        # only print every 100 cycles
        self.intermittent_interval = 100
        self.intermittent_counter = 0
        c = self.detector
        assert isinstance(c,list) and len(c) == 2, c
        self.log("new detector config: %s"%(str(c)) )

        if self.use_propagation:
            # currently velocity not returned by drive service...
            # need to implement that
            raise RuntimeError("'Use Propagation' is not yet implemented.")
            
            # try connecting to the drive service so we can determine the current velocity
            self.drive = self.FindAndConnect("Duckiebot.Drive.Drive")

        # Find and connect to the image service
        self.cam = self.FindAndConnect("Duckiebot.Camera.Camera")
        
        try:
            self.cam.changeFormat('jpeg')
        except: 
            self.cam.changeFormat('bgr')

        # connect to the pipe
        self.imstream = self.cam.ImageStream.Connect(-1) # connect to the pipe
        self.imstream.PacketReceivedEvent+=self._cbImage

        try:
            self.cam.startCapturing()
        except: pass
        
    
    def initializeParams(self,configuration):
        # load params
        param_names = [
            'mean_d_0',
            'mean_phi_0',
            'sigma_d_0',
            'sigma_phi_0',
            'delta_d',
            'delta_phi',
            'd_max',
            'd_min',
            'phi_min',
            'phi_max',
            'cov_v',
            'cov_omega',
            'linewidth_white',
            'linewidth_yellow',
            'lanewidth',
            'min_max',
            'use_min_segs',
            'min_segs',
            'use_max_segment_dist',
            'max_segment_dist',
            'use_distance_weighting',
            'zero_val',
            'l_peak',
            'peak_val',
            'l_max',
            'use_propagation',
            'sigma_d_mask',
            'sigma_phi_mask',

            'img_size',
            'top_cutoff',
            'detector'
            ]

        Configurable.__init__(self,param_names,configuration)
        
        self.mean_0 = [self.mean_d_0, self.mean_phi_0 ]
        self.cov_0  = [ [self.sigma_d_0 , 0] , [0, self.sigma_phi_0] ]
        
        self.cov_mask = [self.sigma_d_mask , self.sigma_phi_mask]

        self.dwa = -(self.zero_val*self.l_peak**2 + self.zero_val*self.l_max**2 - self.l_max**2*self.peak_val - 2*self.zero_val*self.l_peak*self.l_max + 2*self.l_peak*self.l_max*self.peak_val)/(self.l_peak**2*self.l_max*(self.l_peak - self.l_max)**2)
        self.dwb = (2*self.zero_val*self.l_peak**3 + self.zero_val*self.l_max**3 - self.l_max**3*self.peak_val - 3*self.zero_val*self.l_peak**2*self.l_max + 3*self.l_peak**2*self.l_max*self.peak_val)/(self.l_peak**2*self.l_max*(self.l_peak - self.l_max)**2)
        self.dwc = -(self.zero_val*self.l_peak**3 + 2*self.zero_val*self.l_max**3 - 2*self.l_max**3*self.peak_val - 3*self.zero_val*self.l_peak*self.l_max**2 + 3*self.l_peak*self.l_max**2*self.peak_val)/(self.l_peak*self.l_max*(self.l_peak - self.l_max)**2)

#####################################
# RR SERVICE FUNCTIONS / PROPERTIES #
#####################################
    def toggleVerbose(self):
        self._verbose = (not self._verbose)

    @property 
    def verboseImage(self):
        return self._verboseImage
    
    @verboseImage.setter
    def verboseImage(self,value):
        self._verboseImage = value
        self._verboseImagestream = RR.PipeBroadcaster(self._verboseImage,1)

    @property
    def lanePose(self):
        return self._lanePose


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

    def _cbImage(self, pipe_ep):
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

        self.tk = TimeKeeper(image.header)
        self.intermittent_counter += 1

        # extract the image data
        try:
            image_cv = DuckieImageToBGRMat(image)
        except ValueError as e:
            self.log("Could not decode image: %s"%(e))
            return

        self.tk.completed('decode')

        # resize and crop image
        h_original, w_original = image_cv.shape[0:2]

        if self.img_size[0] != h_original or self.img_size[1] != w_original:
            image_cv = cv2.resize(image_cv, (self.img_size[1], self.img_size[0]),
                interpolation=cv2.INTER_NEAREST)
            image_cv = image_cv[self.top_cutoff:,:,:]

        self.tk.completed('resized')
        # apply color correction ... 
        # ADD IN LATER IF NEEDED

        # set the image to be detected
        self.detector.setImage(image_cv)

        # Detect lines and normals
        white = self.detector.detectLines('white')
        yellow = self.detector.detectLines('yellow')
        red = self.detector.detectLines('red')

        self.tk.completed('detected')

        # Reset the segments list
        segmentList = []
        # convert to normalized pixel coordinates, and add segments to segment list
        arr_cutoff = np.array((0, self.top_cutoff, 0, self.top_cutoff))
        arr_ratio = np.array((1./self.img_size[1], 1./self.img_size[0], 1./self.img_size[1], 1./self.img_size[0] ))
        
        if len(white.lines) > 0:
            lines_normalized_white = ((white.lines + arr_cutoff) * arr_ratio)
            segmentList.extend(self.toSegment(lines_normalized_white, white.normals, self.DuckieConsts.WHITE))
        
        if len(yellow.lines) > 0:
            lines_normalized_yellow = ((yellow.lines + arr_cutoff) * arr_ratio)
            segmentList.extend(self.toSegment(lines_normalized_yellow, yellow.normals, self.DuckieConsts.YELLOW))
        
        if len(red.lines) > 0:
            lines_normalized_red = ((red.lines + arr_cutoff) * arr_ratio)
            segmentList.extend(self.toSegment(lines_normalized_red, red.normals, self.DuckieConsts.RED))

        self.intermittent_log('# segments: white %3d yellow %3d red %3d' % (len(white.lines),
                len(yellow.lines), len(red.lines)))

        self.tk.completed('prepared')

        #Publish
        self.processSegments(segmentList)
        self.tk.completed('--processed--')

        # VISUALIZATION
        if self._verbose:

            # Draw lines and normals
            image_with_lines = np.copy(image_cv)
            drawLines(image_with_lines, white.lines, (0,0,0))
            drawLines(image_with_lines, yellow.lines, (255,0,0))
            drawLines(image_with_lines, red.lines, (0,255,0))

            self.tk.completed('drawn')

            # publish the image with lines
            self._pub_im.data = np.reshape(image_with_lines,
                                                 image_with_lines.size)
            self._verboseImagestream.AsyncSendPacket(self._pub_im, lambda:None)
            self.tk.completed('pub_image')

        self.intermittent_log(tk.getall())

    def toSegment(self, lines, normals, color):
        segmentList = []
        for x1,y1,x2,y2,norm_x,norm_y in np.hstack((lines,normals)):
            self.segment.color = color
            self.segment.pixels_normalized[0].x = x1
            self.segment.pixels_normalized[0].y = y1
            self.segment.pixels_normalized[1].x = x2
            self.segment.pixels_normalized[1].y = y2
            self.segment.normal.x = norm_x
            self.segment.normal.y = norm_y
            self.segment.points[0] = self.image2ground(self.pixels_normalized[0])
            self.segment.points[1] = self.image2ground(self.pixels_normalized[1])
            segmentList.append(segment)
        return segmentList
    
    def image2ground(self, pix):
        try: 
            u = pix.u
            v = pix.v
        except AttributeError:
            # we must have been passed a vector
            pix = self.vector2pixel(pix)
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

    def rectifyCVPoint(self,raw2d):
        src_pt = raw2d.reshape((1,1,2)).astype(np.float64)
        rectified = cv2.undistortPoints(src_pt, self.cam_info.K, self.cam_info.D,R=self.cam_info.R, P=self.cam_info.P)
        return rectified

    def processSegments(self,segment_list):
        """
    
Lane Filter Implementation

Author: Liam Paull

Inputs: SegmentList from line detector

Outputs: LanePose - the d (lateral displacement) and phi (relative angle) 
of the car in the lane

For more info on algorithm and parameters please refer to the google doc:
 https://drive.google.com/open?id=0B49dGT7ubfmSX1k5ZVN1dEU4M2M

        """
        if not self.active:
            return
        t_start = time.time()

        if self.use_propagation:
            self.propagateBelief()
            self.t_last_update = time.time()

        # initialize measurement likelihood
        measurement_likelihood = np.zeros(self.d.shape)

        for segment in segment_list:
            if segment.color != self.RRConsts.WHITE and segment.color != self.RRConsts.YELLOW:
                continue
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue

            d_i,phi_i,l_i = self.generateVote(segment)
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i>self.phi_max:
                continue
            if self.use_max_segment_dist and (l_i > self.max_segment_dist):
                continue

            i = floor((d_i - self.d_min)/self.delta_d)
            j = floor((phi_i - self.phi_min)/self.delta_phi)

            if self.use_distance_weighting:           
                dist_weight = self.dwa*l_i**3+self.dwb*l_i**2+self.dwc*l_i+self.zero_val
                if dist_weight < 0:
                    continue
                measurement_likelihood[i,j] = measurement_likelihood[i,j] + dist_weight
            else:
                measurement_likelihood[i,j] = measurement_likelihood[i,j] +  1/(l_i)


        if np.linalg.norm(measurement_likelihood) == 0:
            return
        measurement_likelihood = measurement_likelihood/np.sum(measurement_likelihood)

        if self.use_propagation:
            self.updateBelief(measurement_likelihood)
        else:
            self.beliefRV = measurement_likelihood

        # TODO entropy test:
        #print self.beliefRV.argmax()

        maxids = np.unravel_index(self.beliefRV.argmax(),self.beliefRV.shape)
        
        self._lanePose.d = self.d_min + maxids[0]*self.delta_d
        self._lanePose.phi = self.phi_min + maxids[1]*self.delta_phi
        
        max_val = self.beliefRV.max()
        in_lane = max_val > self.min_max and len(segment_list_msg.segments) > self.min_segs and np.linalg.norm(measurement_likelihood) != 0
        self._lanePose.in_lane = int(in_lane)

        
        # print "time to process segments:"
        # print rospy.get_time() - t_start

    def initializeBelief(self):
        pos = np.empty(self.d.shape + (2,))
        pos[:,:,0]=self.d
        pos[:,:,1]=self.phi
        self.cov_0
        RV = multivariate_normal(self.mean_0,self.cov_0)
        self.beliefRV=RV.pdf(pos)

    def propagateBelief(self):
        delta_t = time.time() - self.t_last_update
        v_current = self.drive.v
        w_current = self.drive.omega

        d_t = self.d + v_current*delta_t*np.sin(self.phi)
        phi_t = self.phi + w_current*delta_t

        p_beliefRV = np.zeros(self.beliefRV.shape)

        for i in range(self.beliefRV.shape[0]):
            for j in range(self.beliefRV.shape[1]):
                if self.beliefRV[i,j] > 0:
                    if d_t[i,j] > self.d_max or d_t[i,j] < self.d_min or phi_t[i,j] < self.phi_min or phi_t[i,j] > self.phi_max:
                        continue
                    i_new = floor((d_t[i,j] - self.d_min)/self.delta_d)
                    j_new = floor((phi_t[i,j] - self.phi_min)/self.delta_phi)
                    p_beliefRV[i_new,j_new] += self.beliefRV[i,j]

        s_beliefRV = np.zeros(self.beliefRV.shape)
        gaussian_filter(100*p_beliefRV, self.cov_mask, output=s_beliefRV, mode='constant')

        if np.sum(s_beliefRV) == 0:
            return
        self.beliefRV = s_beliefRV/np.sum(s_beliefRV)

    def updateBelief(self,measurement_likelihood):
        self.beliefRV=np.multiply(self.beliefRV+1,measurement_likelihood+1)-1
        self.beliefRV=self.beliefRV/np.sum(self.beliefRV)#np.linalg.norm(self.beliefRV)

    def generateVote(self,segment):
        p1 = np.array([segment.points[0].x, segment.points[0].y])
        p2 = np.array([segment.points[1].x, segment.points[1].y])
        t_hat = (p2-p1)/np.linalg.norm(p2-p1)
        n_hat = np.array([-t_hat[1],t_hat[0]])
        d1 = np.inner(n_hat,p1)
        d2 = np.inner(n_hat,p2)
        l1 = np.inner(t_hat,p1)
        l2 = np.inner(t_hat,p2)
        if (l1 < 0):
            l1 = -l1;
        if (l2 < 0):
            l2 = -l2;
        l_i = (l1+l2)/2
        d_i = (d1+d2)/2
        phi_i = np.arcsin(t_hat[1])
        if segment.color == self.RRConsts.WHITE: # right lane is white
            if(p1[0] > p2[0]): # right edge of white lane
                d_i = d_i - self.linewidth_white
            else: # left edge of white lane
                d_i = - d_i
                phi_i = -phi_i
            d_i = d_i - self.lanewidth/2

        elif segment.color == self.RRConsts.YELLOW: # left lane is yellow
            if (p2[0] > p1[0]): # left edge of yellow lane
                d_i = d_i - self.linewidth_yellow
                phi_i = -phi_i
            else: # right edge of white lane
                d_i = -d_i
            d_i =  self.lanewidth/2 - d_i

        return d_i, phi_i, l_i

    def getSegmentDistance(self, segment):
        x_c = (segment.points[0].x + segment.points[1].x)/2
        y_c = (segment.points[0].y + segment.points[1].y)/2

        return sqrt(x_c**2 + y_c**2)

    def onShutdown(self):
        self.log("Shutdown.")


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Initialize the laneInfo service')
    parser.add_argument('--config', type=open,
        help='A config file for the laneInfo service (Otherwise use Default)')
    parser.add_argument('--port',type=int,default=0,
        help='TCP port to host service on' +\
        '(will auto-generate if not specified)')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])

    config_file = args.config
    if config_file is None:
        config_file = '${DEFAULT_LANE_PARAMS}'

    launch_file="""\
node_name: Duckiebot.LaneInfo

objects:
    - name: Duckiebot
      robdef: ${DUCKIEBOT_ROBDEF}
    - name: LaneInfo
      robdef: ${LANEINFO_ROBDEF}
      class: LaneInfoNode.LaneInfoNode
      configuration: %s

tcp_port: %d
    """%(config_file, args.port)

    launch_config = yaml.load(launch_file)

    LaunchRRNode(**launch_config)
