#!/usr/bin/env python
from duckie_utils.configurable import Configurable
from duckie_utils.stats import Stats
import numpy as np
import time
import yaml
import sys, argparse
from math import floor, atan2, pi, cos, sin, sqrt
from scipy.ndimage.filters import gaussian_filter
from scipy.stats import multivariate_normal, entropy

from rr_utils import (RRNodeInterface, LaunchRRNode)
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
        
        
        self.stats = Stats()
        # only print every 100 cycles
        self.intermittent_interval = 100
        self.intermittent_counter = 0

        if self.use_propagation:
            # currently velocity not returned by drive service...
            # need to implement that
            raise RuntimeError("'Use Propagation' is not yet implemented.")
            
            # try connecting to the drive service so we can determine the current velocity
            self.drive = self.FindAndConnect("Duckiebot.Drive.Drive")

        # Find the ground projection service and add a callback for new segments
        self.lineDetector = self.FindAndConnect("Duckiebot.LineDetector.LineDetector")

        # Add the callback to process new segments
        self.lineDetector.newSegments += self.processSegments
        
    
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
            'sigma_phi_mask'
            ]

        Configurable.__init__(self,param_names,configuration)
        
        self.mean_0 = [self.mean_d_0, self.mean_phi_0 ]
        self.cov_0  = [ [self.sigma_d_0 , 0] , [0, self.sigma_phi_0] ]
        
        self.cov_mask = [self.sigma_d_mask , self.sigma_phi_mask]

        self.dwa = -(self.zero_val*self.l_peak**2 + self.zero_val*self.l_max**2 - self.l_max**2*self.peak_val - 2*self.zero_val*self.l_peak*self.l_max + 2*self.l_peak*self.l_max*self.peak_val)/(self.l_peak**2*self.l_max*(self.l_peak - self.l_max)**2)
        self.dwb = (2*self.zero_val*self.l_peak**3 + self.zero_val*self.l_max**3 - self.l_max**3*self.peak_val - 3*self.zero_val*self.l_peak**2*self.l_max + 3*self.l_peak**2*self.l_max*self.peak_val)/(self.l_peak**2*self.l_max*(self.l_peak - self.l_max)**2)
        self.dwc = -(self.zero_val*self.l_peak**3 + 2*self.zero_val*self.l_max**3 - 2*self.l_max**3*self.peak_val - 3*self.zero_val*self.l_peak*self.l_max**2 + 3*self.l_peak*self.l_max**2*self.peak_val)/(self.l_peak*self.l_max*(self.l_peak - self.l_max)**2)

    @property
    def lanePose(self):
        return self._lanePose

    def intermittent_log_now(self):
        return self.intermittent_counter % self.intermittent_interval == 1

    def intermittent_log(self,s):
        if not self.intermittent_log_now():
            return
        msg = "%3d:%s"%(self.intermittent_counter, s)
        self.log(msg)

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
