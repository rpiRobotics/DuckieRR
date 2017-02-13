import cv2
import numpy as np

def DuckieImageToBGRMat(duckieim):
    """
    Take the DuckieImage structure, reshape it, and convert from given format to a CV BGR Mat
    """
    fmt = duckieim.format
    if (fmt == 'bgr'):
        frame=duckieim.data.reshape([duckieim.height, duckieim.width, 3], order='C')
    else:
        if (fmt == 'rgb'):
            frame=duckieim.data.reshape([duckieim.height, duckieim.width, 3], order='C')
            frame=cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif (fmt == 'jpeg'):
            s=np.fromstring(duckieim.data, np.uint8)
            frame = cv2.imdecode(s,cv2.CV_LOAD_IMAGE_COLOR)
            if frame is None:
                msg = 'Could not decode image (cv2.imdecode returned None). '
                msg += 'This is usual a sign of data corruption.'
                raise ValueError(msg)
        elif (fmt == 'gray'):
            frame=duckieim.data.reshape([duckieim.height, duckieim.width, 2], order='C')
            frame=cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        else:
            msg = "Unsupported format type '%s'. Try changing the camera format."%(fmt)
            raise ValueError(msg)
    return frame

def DuckieImageToGrayMat(duckieim):
    """
    Take the DuckieImage structure. reshape it, and convert from the given format to a CV Gray Mat
    """
    fmt = duckieim.format
    if (fmt == 'gray'):
        frame = duckieim.data.reshape([duckieim.height, duckieim.width], order='C')
    else:
        if(fmt == 'rgb'):
            frame=duckieim.data.reshape([duckieim.height, duckieim.width, 3], order='C')
            frame=cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        elif (fmt == 'bgr'):
            frame=duckieim.data.reshape([duckieim.height, duckieim.width, 3], order='C')
            frame=cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        elif (fmt == 'jpeg'):
            s=np.fromstring(duckieim.data, np.uint8)
            frame = cv2.imdecode(s,cv2.CV_LOAD_IMAGE_GRAYSCALE)
            if frame is None:
                msg = 'Could not decode image (cv2.imdecode returned None). '
                msg += 'This is usual a sign of data corruption.'
                raise ValueError(msg)
        else:
            msg = "Unsupported format type '%s'. Try changing the camera format."%(fmt)
            raise ValueError(msg)
    return frame

def DuckieImageToRGBMat(duckieim):
    """
    Take the DuckieImage structure, reshape it, and convert from given format to a CV RGB Mat
    """
    fmt = duckieim.format
    if (fmt == 'rgb'):
        frame=duckieim.data.reshape([duckieim.height, duckieim.width, 3], order='C')
    else:
        if (fmt == 'bgr'):
            frame=duckieim.data.reshape([duckieim.height, duckieim.width, 3], order='C')
            frame=cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif (fmt == 'jpeg'):
            s=np.fromstring(duckieim.data, np.uint8)
            frame = cv2.imdecode(s,cv2.CV_LOAD_IMAGE_COLOR)
            if frame is None:
                msg = 'Could not decode image (cv2.imdecode returned None). '
                msg += 'This is usual a sign of data corruption.'
                raise ValueError(msg)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        elif (fmt == 'gray'):
            frame=duckieim.data.reshape([duckieim.height, duckieim.width, 2], order='C')
            frame=cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB);
            msg = "Can not convert image type 'gray' to 'rgb'. Try changing the camera format."
            raise ValueError(msg)
            # Actually we probably can... but need to check the size of an image captured as gray
        else:
            msg = "Unsupported format type '%s'. Try changing the camera format."%(fmt)
            raise ValueError(msg)
    return frame