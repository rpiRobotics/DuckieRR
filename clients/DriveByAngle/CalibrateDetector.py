#!/usr/bin/env python
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np
from duckie_utils.image import *
import matplotlib.pyplot as plt
import os, sys, argparse
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
    # SHAPE DETECTION
    # Color image with some kind of color filter would probably work even better
    windowSize = (5,5)
    blurred = cv2.GaussianBlur(gray, windowSize,0)
    maxval = 255 # value to set everything above thresh (we want the dark objects)
    
    threshIm = cv2.threshold(blurred, params['BinaryThresh'], maxval, cv2.THRESH_BINARY_INV)[1]
    cv2.imshow('bin', threshIm)

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

    # create controls to adjust parameters
    # change these later to a param file
    cv2.namedWindow('Control', cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar('HarrisThresh', 'Control', 1, 100, cb_HarrisThresh)
    cv2.createTrackbar('BinaryThresh', 'Control', 80, 255, cb_BinaryThresh)
    cv2.createTrackbar('PolyDPLenThresh', 'Control', 4, 5, cb_PolyDPLenThresh)
    cv2.createTrackbar('AreaThresh', 'Control', 25, 1000, cb_AreaThresh)
    cv2.createTrackbar('AngleThresh', 'Control', 80, 180, cb_AngleThresh)

    verts = None
    while (True):
        gray = DuckieImageToGrayMat(cam.captureImage())

        
        gray_BGR = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        
        detections = detectVehicle(gray)
        if len(detections) > 0:
            # choose the largest rectangle...
            i_max = np.argmax([cv2.contourArea(detections[i]) for i in xrange(len(detections))])
            verts = detections[i_max]

            # draw the contour
            #cv2.drawContours(gray_BGR, [c], -1, (0,255,0),2)
            cv2.drawContours(gray_BGR,[verts], -1, (0,255,0),2)

        cv2.imshow('image',gray_BGR)
        key = cv2.waitKey(30)
        if (key == 13 or key == 10):
            break
    
    cv2.destroyAllWindows()


def getParams(config_file):
    # load default params
    global params
    if config_file is None:
        config_file = open('default.yaml','r')
    
    params = yaml.load(config_file.read())

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

def saveAndSendParams(outfile_raw):
    valid_resp = {'y':True,'n':False,'yes':True,'no':False}
    if outfile_raw is None:
        resp = ''
        while resp not in valid_resp:
            resp = raw_input('Do you want to save this calibration? (y/n) > ').lower()
        if valid_resp[resp]:
            outfile_raw = raw_input('Enter a filename > ')
        else:
            return

    outfile = outfile_raw.split('.')[0] + '.yaml'

    with open(outfile,'w') as fout:
        yaml.dump(params,fout,default_flow_style=False)
        fout.close()

        resp=''
        while resp not in valid_resp:
            resp = raw_input('Do you want to transfer this to the Duckiebot? (y/n) > ').lower()
        if valid_resp[resp]:
            os.system('scp "%s" "%s:%s"' %(outfile, 'duckiebot1','/home/ubuntu/DuckieRR/clients/DriveByAngle'))
        


    

#########################
#         MAIN
#########################
# Define a bunch of globals
params = None
cam = None

im_w = 0
im_h = 0
c0 = 0
r0 = 0  

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Run the CalibrateDetector Script')
    parser.add_argument('--config', type=open, 
        help='A config file for internal params (Otherwise use default.yaml)')
    parser.add_argument('--outfile', type=str)
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])

    RRN.UseNumPy = True

    # get the params
    getParams(args.config)

    # Connect to the camera
    cam = RRN.ConnectService("rr+tcp://duckiebot1.local:1235/?service=Camera")
    if cam.format != 'gray':
        cam.changeFormat('gray')
    im_w = cam.resolution[0]
    im_h = cam.resolution[1]
    
    # image center point
    c0 = im_w/2
    r0 = im_h/2

    # Show detection and capture the desired distance
    get_initial_detection()

    # Save the params
    saveAndSendParams(args.outfile)

