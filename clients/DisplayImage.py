#!/usr/bin/env python
from RobotRaconteur.Client import *
from duckie_utils.image import DuckieImageToBGRMat 
import time
import cv2
import numpy as np
import sys, argparse


current_frame = None
def new_frame(pipe_ep):
    global current_frame
    while (pipe_ep.Available>0):
        image=pipe_ep.ReceivePacket()
        current_frame = DuckieImageToBGRMat(image)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Display an Image Stream from the Duckiebot')
    parser.add_argument('service', help="the service to find / connect to. (e.g. Duckiebot.Camera.Camera)")
    parser.add_argument('pipe', help="the name of the pipe to connect to (must send a Duckiebot.Image)")
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])
    service = args.service

    # try and find the service
    RRN.UseNumPy = True
    print "Trying to Find: %s"%service
    time.sleep(5) # wait a few seconds for service discovery to happen
    res = RRN.FindServiceByType(service,['rr+local','rr+tcp'])
    if (len(res)==0):
        raise RuntimeError('Could not find the specified service: %s'%service)
    else:
        serv = RRN.ConnectService(res[0].ConnectionURL)
        print "Found Service!"

    # try and find the topic
    pipe = args.pipe
    pipeObj = getattr(serv,pipe)
    
    #connect to the pipe
    img = pipeObj.Connect(-1)
    img.PacketReceivedEvent+=new_frame

    # open up a new window
    windowName = service+'.'+pipe
    cv2.namedWindow(windowName, cv2.WINDOW_AUTOSIZE)
    
    while True:
        if (not current_frame is None):
            cv2.imshow(windowName, current_frame)
        key = cv2.waitKey(50)
        if (key == 13 or key == 10):
            print "Quitting"
            break

    cv2.destroyAllWindows()
    img.Close()

    