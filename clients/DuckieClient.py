#!/usr/bin/env python
from RobotRaconteur.Client import *
import time
import cv2
import numpy as np

current_frame = None

def DuckieImageToMat(dim):
    """
    Take the DuckieImage structure and reshape it to a CV Mat
    """
    frame2=dim.data.reshape([dim.height, dim.width, 3], order='C')
    frame2=cv2.cvtColor(frame2, cv2.COLOR_RGB2BGR)
    return frame2

def new_frame(pipe_ep):
    """
    This function gets called when a new image is recieved
    """
    global current_frame
    
    #Loop to get the newest frame
    while (pipe_ep.Available > 0):
        #Receive the packet
        image=pipe_ep.ReceivePacket()
        #Convert the packet to an image and set the global variable
        current_frame=DuckieImageToMat(image)


if __name__ == '__main__':
    RRN.UseNumPy = True

    ################
    # Wheels Tests #
    ################
    duckie = RRN.ConnectService("rr+tcp://duckiebot1.local:1234/?service=Drive")
    #duckie = RRN.ConnectService("rr+tcp://10.42.0.48:1234/?service=Drive")
    
    print "Set the gain and trim values (determined experimentally so that duckie drives straight)"
    duckie.gain = 5.0
    duckie.trim = 0.001
    print "gain=%f, trim=%f"%(duckie.gain, duckie.trim)

    print "Reset the parameters to their defaults"
    duckie.resetParams()
    print "gain=%f, trim=%f"%(duckie.gain, duckie.trim) 

    print "Drive forward with a car command (vel, omega)"
    duckie.carCmd(1.0,0.0)
    time.sleep(2)
    print "Send another command to stop"
    duckie.carCmd(0.0,0.0) # stop

    print "Control the wheel velocities directly (vL, vR)"
    duckie.wheelCmd(1.0,-1.0);
    time.sleep(2)
    print "Toggle the EStop"
    duckie.toggleEStop() # stop with the eStop
    print "Duckie should have stopped"
    raw_input('Did the duckie stop? (Press enter to continue)')

    print "Try to move again..."
    duckie.wheelCmd(-1.0,1.0)
    time.sleep(2)
    duckie.wheelCmd(0,0)
    print "Nothing happened because the eStop was toggled"

    print "Release the eStop"
    duckie.toggleEStop()
    print "Try to move again..."
    duckie.wheelCmd(-1.0,1.0)
    time.sleep(2)
    duckie.wheelCmd(0,0)

    ################
    # Camera Tests #
    ################
    cam = RRN.ConnectService("rr+tcp://duckiebot1.local:1235/?service=Camera")

    print "Changing the format to rgb"
    cam.changeFormat('rgb');

    print "Capturing an image"
    cv2.namedWindow("Image")
    frame = DuckieImageToMat(cam.captureImage())
    cv2.imshow("Image", frame)
    cv2.waitKey(1000)

    print  "Capturing 10 images."
    for i in range(10):
        start = time.time()
        frame = DuckieImageToMat(cam.captureImage());
        cv2.imshow("Image", frame)
        print "Elapsed time is %f seconds"%(time.time()-start)
        cv2.waitKey(30)
    

    # set up the stream from the pipe
    print "Connect to the stream from the pipe..."
    cam.toggleFramerate() # set framerate to 15fps 

    s = cam.ImageStream.Connect(-1) # connect to the pipe
    s.PacketReceivedEvent+=new_frame
    
    try: 
        cam.startCapturing(); # start the stream
    except: pass # ignore any errors
    
    
    print "Press any key to exit..."
    while True:
        #Just loop resetting the frame
        #This is not ideal but good enough for demonstration

        if (not current_frame is None):
            cv2.imshow("Image",current_frame)
        if cv2.waitKey(30)!=-1:
            break
    cv2.destroyAllWindows()

    print "Close the pipe and stop capturing"
    s.Close() # close the pipe
    cam.stopCapturing() #stop capturing