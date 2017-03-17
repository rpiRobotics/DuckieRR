#!/usr/bin/env python
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np
from duckie_utils.image import *
import sys, argparse
import yaml
import thread

# Define a bunch of globals
nodetect_limit = 50

wn_divide = 5; zeta = 10

acc_min = -1.0; acc_max = 1.0

vel_min = -0.5; vel_max = 1.0

alpha_d = 0.0
K1 = 1.0 #2.5
K2 = -2.0

K_omg = 0.95

framerate = 10
ifs = 1.0/framerate

im_w = 0; im_h = 0
c0 = 0; r0 = 0  

keypress = False
params = None

cam = None
drive = None

log_time = []
log_vel = []
log_omg = []
log_acc = []

D = np.array([],dtype=np.float64)
K = np.array([],dtype=np.float64)
R = np.array([],dtype=np.float64)
P = np.array([],dtype=np.float64)
map1 = np.ndarray(shape=(480,640, 1),dtype=np.float32)
map2 = np.ndarray(shape=(480,640, 1),dtype=np.float32)

def angle_cos(p0,p1,p2):
    '''
                       |u . v|
    Return cos(th) = -----------
                     ||u|| ||v||
    '''
    d1,d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1,d2)/np.sqrt(np.dot(d1,d1)*np.dot(d2,d2)))

def detectVehicle(grayRaw):
    '''
    Image processing to reliably extract the square tag in an image.
    '''
    # Remap the distorted image.
    gray = np.ndarray(shape=grayRaw.shape,dtype=np.uint8)
    cv2.remap(grayRaw,map1,map2,cv2.INTER_LINEAR,gray)

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
    print "When finished press <ENTER>" 
    print "(Note that the car will start moving if wheels enabled)"

    # create controls to adjust parameters
    # change these later to a param file
    global keypress
    keypress = False
    thread.start_new_thread(keyboard_input_thread, ())

    verts = None
    while (True):
        # if there were other things going on, we should subscribe to a stream
        # but since this is the only thing using the camera, its ok
        grayRaw = DuckieImageToGrayMat(cam.captureImage())
        
        detections = detectVehicle(grayRaw)
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

    f = 1; # the focal length of the camera technically... but it doesn't really matter
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
    
    global keypress
    keypress = False
    thread.start_new_thread(keyboard_input_thread,())
    
    while (not keypress):
        tic = time.time()

        # if there were other things going on, we should subscribe to a stream
        # but since this is the only thing using the camera, its ok
        grayRaw = DuckieImageToGrayMat(cam.captureImage())
         
        detections = detectVehicle(grayRaw)
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
                cX = int(M["m10"]/M["m00"]) #col
                cY = int(M["m01"]/M["m00"]) #row

            f = 1; # the focal length of the camera technically... but it doesn't really matter
            alpha = 2*np.arctan(w/(2*f))


        else:
            if nodetect_count < nodetect_limit:
                #alpha = alpha_prev
                alpha = alpha_d
                vel = 0 # force the integration back to zero. 
                #cX = c0;
                print "no tag %d"%(nodetect_count)
                nodetect_count += 1    
            else:
                print 'WARNING: No tag detected for %d frames. Stopping'%(nodetect_limit) 
                break

        # compute the derivative
        alpha_dot = (2./ifs)*(alpha-alpha_prev)/(alpha+alpha_prev)


        # save the last value
        alpha_prev = alpha 
        
        # Determine controller action
        ang_err = (1.0/alpha)-(1.0/alpha_d)

        acc = K1*ang_err + K2*alpha_dot
        acc = np.clip(acc,acc_min,acc_max)

        vel += acc*ifs
        vel = np.clip(vel, vel_min, vel_max)
        
        # determine the steering controller accuracy
        c_err = float((c0-cX))/(im_w/2) # normalize
        omg = K_omg*c_err
        
        # LOG EVERYTHING
        log_acc.append(acc)
        log_vel.append(vel)
        log_omg.append(omg)
        log_time.append(time.time())

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
    # write data to log file
    import csv 
    filename = time.strftime("%Y-%m-%d-%H-%M-%S") + '.csv'
    with open(filename, 'wb') as myfile:
        wr = csv.writer(myfile)
        wr.writerow(data.keys())
        wr.writerows(zip(*data.values()))



def getParams(config_file):
    # load default params
    global params,nodetect_limit, K1, K_omg, K2, framerate,ifs
    global D, K, R, P
    global wn_divide, zeta
    if config_file is None:
        config_file = open('default.yaml','r')
    
    params = yaml.load(config_file.read())
    nodetect_limit = params["nodetect_limit"]
    wn_divide = params["wn_divide"]
    zeta = params["zeta"]
    K1 = params["K1"]
    K2 = params["K2"]
    K_omg = params["K_omg"]
    framerate = params["framerate"]
    ifs = 1.0/framerate
    d = params['distortion_coefficients']
    D =np.array( d['data'], dtype=np.float64 ).reshape((d['rows'],d['cols']))
    
    k = params['camera_matrix']
    K =np.array( k['data'], dtype=np.float64 ).reshape((k['rows'],k['cols']))
    
    r = params['rectification_matrix']
    R =np.array( r['data'], dtype=np.float64 ).reshape((r['rows'],r['cols']))
    
    p = params['projection_matrix']
    P =np.array( p['data'], dtype=np.float64 ).reshape((p['rows'],p['cols']))

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
    cv2.initUndistortRectifyMap(K,D,R,P,(640,480),cv2.CV_32FC1, map1, map2)


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
    drive.trim = -0.02
    drive.gain = 2.0

    # Show detection and capture the desired distance
    alpha_d = get_initial_detection()
   
    K1 = (2*np.pi*framerate/wn_divide)**2
    K2 = -2*zeta*np.sqrt(K1)/alpha_d**2 

    # Run the main loop
    run_main_loop()

    # Stop the wheels
    drive.carCmd(0,0)

