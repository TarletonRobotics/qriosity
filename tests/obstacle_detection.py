import numpy as np 
import cv2
import time

def drawFlowMap(frame, flow, step=16):
    h, w = frame.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2, -1)
    fx, fy = flow[y, x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines=  np.int32(lines + 0.5)
    vis = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (255, 0, 255), -1)
    return vis, lines

def detect_obstacle_regions(flow):
    print(flow)

if __name__ == '__main__':
    print(cv2.__version__)
    # open camera device / video file
    cam = cv2.VideoCapture('/home/michael/Code/qriosity/tests/unsw_run.mp4')
    # get first frame,
    _, oldFrame = cam.read()
    grayOldFrame = cv2.cvtColor(oldFrame, cv2.COLOR_BGR2GRAY)

    # start render loop
    while True:
        # read a frame from image
        ret, newFrame = cam.read()
        if ret:
            # grayscale image.
            grayNewFrame = cv2.cvtColor(newFrame, cv2.COLOR_BGR2GRAY)
            # compute optical flow field.
            flow = cv2.calcOpticalFlowFarneback(
                grayOldFrame,grayNewFrame, None, 0.5, 3, 15, 3, 5, 1.2, 0)

            # swap old frame for new f
            grayOldFrame = grayNewFrame
            # show optical flow field.
            flowMap, flowArray = drawFlowMap(grayNewFrame, flow)
            
            detect_obstacle_regions(flowArray)
            
            cv2.imshow('optical flow', flowMap)
            # quit
        ch = 0xFF & cv2.waitKey(5)
        if ch == 27:
            break
    cv2.destroyAllWindows()     