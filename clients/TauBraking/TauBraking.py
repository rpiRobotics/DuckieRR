#!/usr/bin/env python
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np
from duckie_utils.image import *
#import matplotlib.pyplot as plt
import sys, argparse
import yaml
import thread

# Define a bunch of globals
nodetect_limit = 15

acc_min = -1.0
acc_max = 1.0

vel_min = -0.5
vel_max = 1.0

w_d = 0.0
tau_th = 1.8
tau_dot_d = -0.5

Kp = 1.0 #2.5

Kp_omg = 0.95

framerate = 15
ifs = 1.0/framerate

im_w = 0
im_h = 0
c0 = 0
r0 = 0  

keypress = False
params = None

cam = None
drive = None


def donothing(x):
    pass

def angle_cos(p0,p1,p2):
    '''
                       |u . v|
    Return cos(th) = -----------
                     ||u|| ||v||
    '''
    d1,d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1,d2)/np.sqrt(np.dot(d1,d1)*np.dot(d2,d2)))

def detectVehicle(gray):
    '''
    Image processing to reliably extract the square tag in an image.
    '''
    
    '''
    # CORNER DETECTION
    # not that useful for real images...
    
    # Detect corners
    gray_flt = np.float32(gray)
    # blockSize - size of window / neighborhood
    # ksize - Aperature param of Sobel derivative
    # k - Harris detector free parameter
    corners = cv2.cornerHarris(gray_flt, blockSize=2, ksize=3, k=0.04)
    corners = cv2.dilate(corners,None) 

    gray_BGR[corners>params['HarrisThresh']*corners.max()] = [0,0,255]
    '''

    # SHAPE DETECTION
    # Color image with some kind of color filter would probably work even better
    windowSize = (5,5)
    blurred = cv2.GaussianBlur(gray, windowSize,0)
    maxval = 255 # value to set everything above thresh (we want the dark objects)
    
    '''
    THRESHOLD
        ret, binIm = threshold(im, th, maxval, mode)
    INPUTS:   
        im:         input image
        th:         threshold value
        maxval:     value to set everything that is below thresh   
        mode:       thresholding mode
                    THRESHOLD_BINARY - create a binary image (everything below thresh=0)
                    THRESHOLD_BINARY_INV - create a binary image (everything above thresh=0)
    OUTPUTS:
        ret:        Return code
        binIm:      the binary image created by thresholding
    '''
    threshIm = cv2.threshold(blurred, params['BinaryThresh'], maxval, cv2.THRESH_BINARY_INV)[1]
    '''
    FINDCONTOURS
        im_new, contours, hierarchy  = findContours(im, ret_mode, approx_meth:
    INPUTS:
        im:             input image. Is altered by the function, create copy to preserve original
        ret_mode:       contour retreival mode 
                        RETR_EXTERNAL - retrieve only the extreme outer contours
        approx_meth:    countour approx method
                        CHAIN_APPROX_SIMPLE - encode only end points of segments
    OUTPUTS:
        im_new:         new binary image
        contours:       vector of countour points
        hierarchy:      how the vector is structured.
    '''
    contours = cv2.findContours(threshIm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[1] 
    
    detections = []
    # loop over the contours
    for c in contours:
        # determine the length of the contour (around the perimeter)
        c_len = cv2.arcLength(c,True)

        # Try to fit to a polygonal curve with some margin of error(0-5%) from the original 
        verts = cv2.approxPolyDP(c, params['PolyDPLenThresh']*c_len, True)
        
        # Determine the area inside the contour.
        area = cv2.contourArea(verts)
        
        # if we have 4 vertices and the area isn't too small, we have a quadrilateral
        if len(verts) == 4 and area>params['AreaThresh']: 
            # reshape vertices to 2 columns (infer num rows)
            verts = verts.reshape(-1,2)
            
            # evaluate cos(th) for all sets of angles... pick the largest...
            max_cos = np.max([angle_cos(verts[i], verts[(i+1)%4], verts[(i+2)%4]) for i in xrange(4)])

            # if our largest angle_cos is less than the angleCosThresh ( ~angle_cos(90 deg) ) then we have a rectangle 
            AngleCosThres = np.cos(params['AngleThresh'])
            if max_cos < AngleCosThres:
                detections.append(np.array(verts))
    return detections

def get_initial_detection():
    print "Initial Detection routine..."
    print "Place vehicle at desired stop distance"
    print "When finished press <ENTER>" 

    # create controls to adjust parameters
    # change these later to a param file
    global keypress
    keypress = False
    thread.start_new_thread(keyboard_input_thread, ())

    verts = None
    while (True):
        # if there were other things going on, we should subscribe to a stream
        # but since this is the only thing using the camera, its ok
        gray = DuckieImageToGrayMat(cam.captureImage())
        
        detections = detectVehicle(gray)
        if len(detections) > 0:
            # choose the largest rectangle...
            i_max = np.argmax([cv2.contourArea(detections[i]) for i in xrange(len(detections))])
            verts = detections[i_max]

            print verts

        if keypress:
            break

    if verts is None:
        raise RuntimeError('No Tag Detected')
    
    w = np.max(verts[:,0]) - np.min(verts[:,0])
    
    return w # We will use the width instead of the optical angle...

def run_main_loop():
    acc = 0.0
    vel = 0.0
    omg = 0.0
    framenum = 0
    avg_time = 0.0
    
    w_prev = w_d

    tau_prev = np.Inf
    tau_dot_list = np.zeros(5)
    cX = c0
    nodetect_count = 0
    
    global keypress
    keypress = False
    thread.start_new_thread(keyboard_input_thread,())
    
    while (not keypress):
        tic = time.time()

        # if there were other things going on, we should subscribe to a stream
        # but since this is the only thing using the camera, its ok
        gray = DuckieImageToGrayMat(cam.captureImage())
        
        detections = detectVehicle(gray)
        if len(detections) > 0:
            nodetect_count = 0
            # choose the largest rectangle...
            i_max = np.argmax([cv2.contourArea(detections[i]) for i in xrange(len(detections))])
            verts = detections[i_max]
            
            w = np.max(verts[:,0]) - np.min(verts[:,0])

            # compute the center of the contour
            M = cv2.moments(verts)
            if (M["m00"]!=0):
                cX = int(M["m10"]/M["m00"]) #col
                cY = int(M["m01"]/M["m00"]) #row

        else:
            if nodetect_count < nodetect_limit:
                #alpha = alpha_prev
                w = w_d
                vel = 0 # force the integration back to zero. 
                #cX = c0;
                print "no tag %d"%(nodetect_count)
                nodetect_count += 1    
            else:
                print 'WARNING: No tag detected for %d frames. Stopping'%(nodetect_limit) 
                break

        w_dot = (w-w_prev)/ifs # not generally good practice....
        if w_dot != 0:
            tau = (w_d-w)/w_dot
        else:
            tau = tau_prev

        # compute the derivative
        tau_dot = (tau-tau_prev)/ifs # again generally not good practice
        
        # keep the last 5 values
        tau_dot_list[framenum%5] = tau_dot

        if framenum < 4:
            tau_dot_avg = tau_dot
        else:
            # implement a moving average
            tau_dot_avg = np.mean(tau_dot_list)

        # save the last value
        w_prev = w 
        
        # Determine controller action
        if tau > tau_th:
            vel = vel_max
        else:
            # WE NEED TO BRAKE!
            tau_dot_err = (tau_dot_d-tau_dot_avg)
            acc = Kp*tau_dot_err
            acc = np.clip(acc, acc_min,acc_max)
            vel += acc*ifs

        vel = np.clip(vel, vel_min, vel_max)

        
        # determine the steering controller accuracy
        c_err = float((c0-cX))/(im_w/2) # normalize
        omg = Kp_omg*c_err
        
        # SEND THE COMMAND TO THE MOTORS
        drive.carCmd(vel, omg)        
        
        # increment the frame number
        framenum += 1

        toc = time.time() - tic
        if toc < ifs:
            time.sleep(ifs-toc)
        
        toc2 = time.time()-tic
        avg_time = ((framenum-1)*avg_time/framenum) + (toc2/framenum)

        
    print "Average Loop Freq: %f"%(1.0/avg_time)


def getParams(config_file):
    # load default params
    global params,nodetect_limit, Kp, Kp_omg, framerate,ifs
    if config_file is None:
        config_file = open('default.yaml','r')
    
    params = yaml.load(config_file.read())
    nodetect_limit = params["nodetect_limit"]
    Kp = params["Kp"]
    Kp_omg = params["Kp_omg"]
    framerate = params["framerate"]
    ifs = 1.0/framerate

def keyboard_input_thread():
    global keypress
    raw_input('press enter to stop')
    keypress = True


#########################
#         MAIN
#########################

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run the DriveByAngle Script')
    parser.add_argument('--config', type=open, 
        help='A config file for internal params (Otherwise use default.yaml)')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])

    RRN.UseNumPy = True

    # get the params
    getParams(args.config)

    # Connect to the camera
    cam = RRN.ConnectService("rr+local:///?nodename=Duckiebot.Camera&service=Camera")
    if cam.format != 'gray':
        cam.changeFormat('gray')
    im_w = cam.resolution[0]
    im_h = cam.resolution[1]
    
    # image center point
    c0 = im_w/2
    r0 = im_h/2

    # Connect to the wheels
    drive = RRN.ConnectService("rr+local:///?nodename=Duckiebot.Drive&service=Drive")
            
    vel_max = drive.limit
    #drive.trim = -0.02
    #drive.gain = 1.5

    # Show detection and capture the desired distance
    w_d = get_initial_detection()
    
    msg = "place the vehicle at the starting mark and press enter when ready.\n"
    msg += "(note that the vehicle will start moving!)"
    raw_input(msg) 
    # Run the main loop
    run_main_loop()

    # Stop the wheels
    drive.carCmd(0,0)

