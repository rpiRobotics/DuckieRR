#!/usr/bin/env python
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np
from duckie_utils.image import *
import matplotlib.pyplot as plt
import sys, argparse
import yaml
import thread


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
    if debug:
        cv2.imshow('bin', threshIm)

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
    print "When finished press <ENTER>" 
    print "(Note that the car will start moving if wheels enabled)"

    # create controls to adjust parameters
    # change these later to a param file
    if onDuckie:
	global keypress
        keypress = False
        thread.start_new_thread(keyboard_input_thread, ())
    else:
        cv2.namedWindow('Control', cv2.WINDOW_AUTOSIZE)
        cv2.createTrackbar('HarrisThresh', 'Control', 1, 100, cb_HarrisThresh)
        cv2.createTrackbar('BinaryThresh', 'Control', 80, 255, cb_BinaryThresh)
        cv2.createTrackbar('PolyDPLenThresh', 'Control', 4, 5, cb_PolyDPLenThresh)
        cv2.createTrackbar('AreaThresh', 'Control', 25, 1000, cb_AreaThresh)
        cv2.createTrackbar('AngleThresh', 'Control', 80, 180, cb_AngleThresh)

    verts = None
    while (True):
        if local:
            ret, frame = cam.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            # if there were other things going on, we should subscribe to a stream
            # but since this is the only thing using the camera, its ok
            gray = DuckieImageToGrayMat(cam.captureImage())
            gray = cv2.undistort(gray,K,D)

        if not onDuckie:
            gray_BGR = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        detections = detectVehicle(gray)
        if len(detections) > 0:
            # choose the largest rectangle...
            i_max = np.argmax([cv2.contourArea(detections[i]) for i in xrange(len(detections))])
            verts = detections[i_max]

            if onDuckie:
                print verts
            else:
                # draw the contour
                #cv2.drawContours(gray_BGR, [c], -1, (0,255,0),2)
                cv2.drawContours(gray_BGR,[verts], -1, (0,255,0),2)

        if onDuckie:
            if keypress:
                break
        else:
            cv2.imshow('image',gray_BGR)
            key = cv2.waitKey(30)
            if (key == 13 or key == 10):
                break
    
    if not onDuckie:
        cv2.destroyWindow('Control')
        if not debug:
            cv2.destroyWindow('image')

    if verts is None:
        raise RuntimeError('No Tag Detected')
    
    w = np.max(verts[:,0]) - np.min(verts[:,0])

    f = 1; # the focal length of the camera technically... but it (probably) doesn't matter
    alpha = 2*np.arctan(w/(2*f))
    
    return alpha

def run_main_loop():
    acc = 0.0
    vel = 0.0
    omg = 0.0
    framenum = 0
    avg_time = 0.0
    alpha_prev = alpha_d
    alpha_dot_list = np.zeros(5)
    cX = c0
    nodetect_count = 0

    while (True):
        tic = time.time()

        if local:
            ret, frame = cam.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            # if there were other things going on, we should subscribe to a stream
            # but since this is the only thing using the camera, its ok
            gray = DuckieImageToGrayMat(cam.captureImage())
            gray = cv2.undistort(gray,K,D)

        if debug:
            gray_BGR = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        detections = detectVehicle(gray)
        if len(detections) > 0:
            nodetect_count = 0
            # choose the largest rectangle...
            i_max = np.argmax([cv2.contourArea(detections[i]) for i in xrange(len(detections))])
            verts = detections[i_max]
            
            w = np.max(verts[:,0]) - np.min(verts[:,0])
            h = np.max(verts[:,1]) - np.min(verts[:,1])

            # compute the center of the contour
            M = cv2.moments(verts)
            if (M["m00"]!=0):
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])
                if debug:
                    cv2.circle(gray_BGR, (cX,cY), 7, (255,255,255),-1)
            
            #else:
            #    cX = cX;

            f = 1; # the focal length of the camera technically... but it (probably) doesn't matter
            alpha = 2*np.arctan(w/(2*f))

            if debug:
                # draw the contour
                #cv2.drawContours(gray_BGR, [c], -1, (0,255,0),2)
                cv2.drawContours(gray_BGR,[verts], -1, (0,255,0),2)
        else:
            if nodetect_count < nodetect_limit:
                #alpha = alpha_prev
                alpha = alpha_d
                #cX = c0;
                print "no tag %d"%(nodetect_count)
                nodetect_count += 1    
            else:
                print 'WARNING: No tag detected for %d frames. Stopping'%(nodetect_limit) 
                break

        # compute the derivative
        alpha_dot = (alpha-alpha_prev)/ifs
        
        # keep the last 5 values
        alpha_dot_list[framenum%5] = alpha_dot

        if framenum < 4:
            alpha_dot_avg = alpha_dot
        else:
            # implement a moving average
            alpha_dot_avg = np.mean(alpha_dot_list)

        # save the last value
        alpha_prev = alpha 
        
        # Determine controller action
        ang_err = (1.0/alpha)-(1.0/alpha_d)

        acc = Kp*ang_err + Kd*alpha_dot_avg
        acc = np.clip(acc,acc_min,acc_max)

        vel += acc*ifs
        vel = np.clip(vel, vel_min, vel_max)
        
        # determine the steering controller accuracy
        c_err = float((c0-cX))/(im_w/2) # normalize
        omg = Kp_omg*c_err

        # update our plot
        if debug:
            update_line(line0,framenum,alpha)
            update_line(line1, framenum, vel)
            update_line(line2, framenum, acc)
            cv2.imshow('image',gray_BGR)
            plt.pause(0.0001)
        
            if (cv2.waitKey(1) == 27):
                break
        
        # SEND THE COMMAND TO THE MOTORS
        if wheels:
            drive.carCmd(vel, omg)

        # increment the frame number
        framenum += 1        
        
        toc = time.time() - tic
        if toc < ifs:
            time.sleep(ifs-toc)

        toc2 = time.time()-tic
        avg_time = ((framenum-1)*avg_time/framenum) + (toc2/framenum)

        
    print "Average Loop Freq: %f"%(1.0/avg_time)

def update_line(h1,new_xdata,new_ydata):
    h1.set_xdata(np.append(h1.get_xdata(), new_xdata))
    h1.set_ydata(np.append(h1.get_ydata(), new_ydata))
    h1.axes.relim()
    h1.axes.autoscale_view()
    plt.draw()

def getParams(config_file):
    # load default params
    global params, D, K, R, P

    if config_file is None:
        config_file = open('default.yaml','r')
    
    params = yaml.load(config_file.read())

    d = params['distortion_coefficients']
    D =np.array( d['data'], dtype=np.float64 ).reshape((d['rows'],d['cols']))
    
    k = params['camera_matrix']
    K =np.array( k['data'], dtype=np.float64 ).reshape((k['rows'],k['cols']))
    
    r = params['rectification_matrix']
    R =np.array( r['data'], dtype=np.float64 ).reshape((r['rows'],r['cols']))
    
    p = params['projection_matrix']
    P =np.array( p['data'], dtype=np.float64 ).reshape((p['rows'],p['cols']))

def updateParam(name, value):
    global params
    params[name] = value

def cb_HarrisThresh(val):
    global params
    updateParam('HarrisThresh',val)
    params['HarrisThresh']/=100.0

def cb_BinaryThresh(val):
    updateParam('BinaryThresh',val)

def cb_PolyDPLenThresh(val):
    updateParam('PolyDPLenThresh',val)
    params['PolyDPLenThresh']/=100.0

def cb_AreaThresh(val):
    global params
    updateParam('AreaThresh',val)
    params['AreaThresh']*=10

def cb_AngleThresh(val):
    updateParam('AngleThresh',val)
    params['AngleThresh']*=(np.pi/180.0)

def keyboard_input_thread():
    global keypress
    raw_input('press enter to stop')
    keypress = True


#########################
#         MAIN
#########################
# Define a bunch of globals
debug = False
local = False
wheels = True
onDuckie = False


keypress = False
params = None
line0 = None
line1 = None
line2 = None
cam = None
drive = None

nodetect_limit = 10

acc_min = -1.0
acc_max = 1.0

vel_min = -0.5
vel_max = 1.0

alpha_d = 0.0
Kp = 2.0
Kd = -2.0

Kp_omg = 0.8;


framerate = 15
ifs = 1.0/framerate

im_w = 0
im_h = 0
c0 = 0
r0 = 0  

D = np.array([],dtype=np.float64)
K = np.array([],dtype=np.float64)
R = np.array([],dtype=np.float64)
P = np.array([],dtype=np.float64)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run the DriveByAngle Script')
    parser.add_argument('--local', action='store_true',
        help='run the code using laptop camera')
    parser.add_argument('--debug', action='store_true',
        help='run the code with additional controls')
    parser.add_argument('--nowheels', action='store_true',
        help='run the code but do not drive the car' )
    parser.add_argument('--onDuckie', action='store_true',
        help='run the code ON the duckiebot')
    parser.add_argument('--config', type=open, 
        help='A config file for internal params (Otherwise use default.yaml)')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])
    debug = args.debug
    local = args.local
    wheels = not args.nowheels
    onDuckie = args.onDuckie

    RRN.UseNumPy = True

    # get the params
    getParams(args.config)


    # Connect to the camera
    if local:
        cam = cv2.VideoCapture(0)
        wheels = False # no matter what the command line said...
        im_w = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
        im_h = cam.get(cv2.CAP_PROP_FRAME_HEIGHT) 

    else:
        if onDuckie:
            cam = RRN.ConnectService("rr+local:///?nodename=Duckiebot.Camera&service=Camera")
        else:
            cam = RRN.ConnectService("rr+tcp://duckiebot1.local:1235/?service=Camera")
        if cam.format != 'gray':
            cam.changeFormat('gray')
        im_w = cam.resolution[0]
        im_h = cam.resolution[1]
        if wheels:
            # Connect to the wheels
            if onDuckie:
                drive = RRN.ConnectService("rr+local:///?nodename=Duckiebot.Drive&service=Drive")
            else:
                drive = RRN.ConnectService("rr+tcp://duckiebot1.local:1234/?service=Drive")
            
            vel_max = drive.limit
            drive.trim = -0.02

    # image center point
    c0 = im_w/2
    r0 = im_h/2

    # Show detection and capture the desired distance
    alpha_d = get_initial_detection()

    # create a figure to plot the angle
    if debug:
        fig0 = plt.figure(0)
        line0, = plt.plot([],[])
        line0.axes.autoscale()

        # create another figure to show the vel and acc
        fig1 = plt.figure(2)
        line1,line2 = plt.plot([],[],[],[])
        line1.axes.autoscale()
        line2.axes.autoscale()
    
        # make the plots interactive and show them
        plt.ion()
        plt.show()
    
    # Run the main loop
    run_main_loop()

    # Update the vel to the wheels
    if wheels:
        drive.carCmd(0,0)

    if local:
        cam.release()

    raw_input('Press enter to exit.')
    cv2.destroyAllWindows()
    plt.close('all')


