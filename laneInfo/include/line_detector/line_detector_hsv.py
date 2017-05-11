from line_detector.line_detector_interface import (Detections, LineDetectorInterface)
import cv2
import numpy as np
from duckie_utils.configurable import Configurable
from duckie_utils.image import is_cv2

class LineDetectorHSV(Configurable, LineDetectorInterface):
    """LineDetector using Hue, Saturation, Value (HSV) color thresholding"""
    def __init__(self, configuration):
        # images to be processed
        self.bgr = np.empty(0)
        self.hsv = np.empty(0)
        self.edges = np.empty(0)

        param_names = [
            'hsv_white1',
            'hsv_white2',
            'hsv_yellow1',
            'hsv_yellow2',
            'hsv_red1',
            'hsv_red2',
            'hsv_red3',
            'hsv_red4',
            'dilation_kernel_size',
            'canny_thresholds',
            'hough_threshold',
            'hough_min_line_length',
            'hough_max_line_gap',
        ]

        Configurable.__init__(self, param_names, configuration)
        
    def _colorFilter(self, color):
        # threshold colors in HSV space
        if color == 'white':
            bw = cv2.inRange(self.hsv, self.hsv_white1, self.hsv_white2)
        elif color == 'yellow':
            bw = cv2.inRange(self.hsv, self.hsv_yellow1, self.hsv_yellow2)
        elif color == 'red':
            bw1 = cv2.inRange(self.hsv, self.hsv_red1, self.hsv_red2)
            bw2 = cv2.inRange(self.hsv, self.hsv_red3, self.hsv_red4)
            bw = cv2.bitwise_or(bw1, bw2)
        else:
            raise Exception('Error: Undefined color strings...')

        # binary dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(self.dilation_kernel_size, self.dilation_kernel_size))
        bw = cv2.dilate(bw, kernel)
        
        # refine edge for certain color
        edge_color = cv2.bitwise_and(bw, self.edges)

        return bw, edge_color

    def _findEdge(self, gray):
        edges = cv2.Canny(gray, self.canny_thresholds[0], self.canny_thresholds[1], apertureSize=3)
        return edges

    def _HoughLine(self, edge):
        if is_cv2():
            lines = cv2.HoughLinesP(edge, 1, np.pi/180, self.hough_threshold, np.empty(1), self.hough_min_line_length, self.hough_max_line_gap)
        else:
            lines = cv2.HoughLinesP(edge, 1, np.pi/180, self.hough_threshold, minLineLength=self.hough_min_line_length, maxLineGap=self.hough_max_line_gap)
        
        if lines is not None:
            if is_cv2():
                lines = np.array(lines[0])
            else:
                lines = np.reshape(lines,(-1,4))
        else:
            lines = []
        return lines

    def _checkBounds(self, val, bound):
        val[val<0]=0
        val[val>=bound]=bound-1
        return val

    def _correctPixelOrdering(self, lines, normals):
        flag = ((lines[:,2]-lines[:,0])*normals[:,1] - (lines[:,3]-lines[:,1])*normals[:,0])>0
        for i in range(len(lines)):
            if flag[i]:
                x1,y1,x2,y2 = lines[i, :]
                lines[i, :] = [x2,y2,x1,y1] 
 
    def _findNormal(self, bw, lines):
        normals = []
        centers = []
        if len(lines)>0:
            length = np.sum((lines[:, 0:2] -lines[:, 2:4])**2, axis=1, keepdims=True)**0.5
            dx = 1.* (lines[:,3:4]-lines[:,1:2])/length
            dy = 1.* (lines[:,0:1]-lines[:,2:3])/length

            centers = np.hstack([(lines[:,0:1]+lines[:,2:3])/2, (lines[:,1:2]+lines[:,3:4])/2])
            x3 = (centers[:,0:1] - 3.*dx).astype('int')
            y3 = (centers[:,1:2] - 3.*dy).astype('int')
            x4 = (centers[:,0:1] + 3.*dx).astype('int')
            y4 = (centers[:,1:2] + 3.*dy).astype('int')
            x3 = self._checkBounds(x3, bw.shape[1])
            y3 = self._checkBounds(y3, bw.shape[0])
            x4 = self._checkBounds(x4, bw.shape[1])
            y4 = self._checkBounds(y4, bw.shape[0])
            flag_signs = (np.logical_and(bw[y3,x3]>0, bw[y4,x4]==0)).astype('int')*2-1
            normals = np.hstack([dx, dy]) * flag_signs
 
 
            self._correctPixelOrdering(lines, normals)
        return centers, normals

    def detectLines(self, color):
        bw, edge_color = self._colorFilter(color)
        lines = self._HoughLine(edge_color)
        centers, normals = self._findNormal(bw, lines)
        return Detections(lines=lines, normals=normals, area=bw, centers=centers)

    def setImage(self, bgr):
        self.bgr = np.copy(bgr)
        self.hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        self.edges = self._findEdge(self.bgr)
  
    def getImage(self):
        return self.bgr

# def _main():
#     import yaml,sys
#     from line_detector.line_detector_plot import (drawLines, drawNormals)
#
#     with open('/home/greg/duckiebot_ws/DuckieRR/laneInfo/config/default_LD.yaml','r') as f:
#         params = yaml.load(f.read())
#    
#     config = params['detector'][1]['configuration']
#     detector = LineDetectorHSV(config)
#     # read image from file or camera
#     if len(sys.argv)==2:
#         bgr = cv2.imread(sys.argv[1])
#
#         # crop and resize frame
#         bgr = cv2.resize(bgr, (200, 150))
#         bgr = bgr[bgr.shape[0]/2:, :, :]
#
#         # set the image to be detected
#         detector.setImage(bgr)
#
#         # detect lines and normals
#         lines_white, normals_white, area_white = detector.detectLines('white')
#         lines_yellow, normals_yellow, area_yellow = detector.detectLines('yellow')
#         lines_red, normals_red, area_red = detector.detectLines('red')
#
#         # draw lines
#         drawLines(detector.bgr,lines_white, (0,0,0))
#         drawLines(detector.bgr,lines_yellow, (255,0,0))
#         drawLines(detector.bgr,lines_red, (0,255,0))
#
#         # draw normals
#         drawNormals(detector.bgr,lines_yellow, normals_yellow)
#         drawNormals(detector.bgr,lines_white, normals_white)
#         drawNormals(detector.bgr,lines_red, normals_red)
#
#         cv2.imwrite('lines_with_normal.png', detector.bgr)
#         cv2.imshow('frame', detector.bgr)
#         cv2.imshow('edge', detector.edges)
#         cv2.waitKey(0)
#
#     elif len(sys.argv)==1:
#         cap = cv2.VideoCapture(0)
#         if not cap.isOpened():
#             print 'Error opening camera...'
#             return -1
#
#         while True:
#             ret, bgr = cap.read()
#             if not ret:
#                 print 'No frames grabbed...'
#                 break
#
#             # crop and resize frame
#             bgr = cv2.resize(bgr, (200, 150))
#
#             # set the image to be detected
#             detector.setImage(bgr)
#
#             # detect lines and normals
#             lines_white, normals_white, centers_white, area_white = detector.detectLines('white')
#             lines_yellow, normals_yellow, centers_yellow, area_yellow = detector.detectLines('yellow')
#             lines_red, normals_red, centers_red, area_red = detector.detectLines('red')
#
#             print "# white: %d; # yellow: %d; # red: %d"%(len(lines_white),len(lines_yellow),len(lines_red))
#           
#             # draw lines
#             drawLines(detector.bgr,lines_white, (0,0,0))
#             drawLines(detector.bgr,lines_yellow, (255,0,0))
#             drawLines(detector.bgr,lines_red, (0,255,0))
#
#             # draw normals
#             drawNormals(detector.bgr,lines_yellow, normals_yellow)
#             drawNormals(detector.bgr,lines_white, normals_white)
#             drawNormals(detector.bgr,lines_red, normals_red)
#
#             # show frame
#             cv2.imshow('Line Detector', detector.bgr)
#             cv2.imshow('Edge', detector.edges)
#             cv2.waitKey(30)
#
#     else:
#         return -1
# if __name__ == '__main__':
#     _main()
