#!/usr/bin/env python
from RobotRaconteur.Client import *
RRN.UseNumPy = True
import time
import cv2
import numpy as np
import sys, argparse
from duckie_utils.stats import Stats

tcam_prev = 0.

def new_packet(pipe):
    global tcam_prev
    while (pipe.Available>0):
        im = pipe.ReceivePacket()
        tr = time.time()
        tcam = im.header.time
        cam_delay = tcam-tcam_prev
        delay = tr-im.header.time
        tcam_prev = tcam
        print "cam delay: %0.3f (%0.3f fps), trans delay: %0.3f (%0.3f fps)"%(cam_delay, 1./cam_delay, delay, 1./delay)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Check the speed of the camera connection')
    parser.add_argument('--format', default='bgr',
                        help='format to request (not guaranteed)')
    parser.add_argument('--tcp', action='store_true', help='check tcp connections as well')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])

    fmt = args.format

    tcp = args.tcp
    connections = ['rr+local']
    if tcp:
        connections.extend('rr+tcp')

    # connect to the service
    print "Trying to find the camera service"
    time.sleep(5)
    res = RRN.FindServiceByType('Duckiebot.Camera.Camera', connections)
    if (len(res)==0):
        raise RuntimeError('Could not find the camera service')
    else:
        cam = RRN.ConnectService(res[0].ConnectionURL)
        print 'Connected to Camera'

    # connect to the pipe
    pipe = cam.ImageStream
    img = pipe.Connect(-1)
    img.PacketReceivedEvent+=new_packet

    try:
        print "Trying to change the camera format to: %s"%(fmt)
        cam.changeFormat(fmt)
    except:
        pass
    print "Cam Format is now: %s"%(cam.format)

    try:
        cam.startCapturing()
    except:
        pass

    try:
        while True:
            time.sleep(0.001)
    finally:
        img.Close()
        time.sleep(1)
        RRN.Shutdown()
