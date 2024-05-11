import cv2
import numpy as np
import time
from picamera2 import Picamera2
# used to record the time when we processed last frame
prev_frame_time = 0

# used to record the time at which we processed current frame
new_frame_time = 0
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

ROI1 = [20, 170, 240, 220]
ROI2 = [400, 170, 620, 220]
ROI3 = [200, 300, 440, 350]
while True:
    im= picam2.capture_array()
    points = [(115,200), (525,200), (640,370), (0,370)]
    width = 640
    height = 480
    input = np.float32(points)
    output = np.float32([(0,0), (width-1,0), (width-1,height-1), (0,height-1)])
    # compute perspective matrix
    matrix = cv2.getPerspectiveTransform(input,output)
    imgPerspective = cv2.warpPerspective(im, matrix, (width,height),
    cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))


    #grayscaling
    imgGray = cv2.cvtColor(imgPerspective, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("gray", imgGray)
    #thresholding
    ret, imgThresh = cv2.threshold(imgGray, 60, 255, cv2.THRESH_BINARY_INV)

    contours, hierarchy = cv2.findContours(imgThresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


    print("Number of Contours found = " + str(len(contours)))
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    
    # black mask
    #lower_black = np.array([0, 0, 0])
    #upper_black = np.array([180, 255, 50])
    
    #imgThresh = cv2.inRange(img_hsv, lower_black, upper_black)
    
    # orange mask
    lower_orange = np.array([0, 100, 20])
    upper_orange = np.array([25, 255, 255])
    o_mask = cv2.inRange(img_hsv, lower_orange, upper_orange)    
    contours_orange = cv2.findContours(o_mask[ROI3[1]:ROI3[3], ROI3[0]:ROI3[2]], cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    #find left and right contours of the lanes
    contours_left, hierarchy = cv2.findContours(imgThresh[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    contours_right, hierarchy = cv2.findContours(imgThresh[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]],cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    #find all contours in image for debugging
    contours, hierarchy = cv2.findContours(imgThresh, 
    cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    #cv2.drawContours(imgPerspective, contours, 1, (0, 255, 0), 2)
    approx=cv2.approxPolyDP(contours_left, 0.01*cv2.arcLength(contours_left,True),True)
    x,y,w,h=cv2.boundingRect(approx)

    cv2.rectangle(imgPerspective,(x,y),(x+w,y+h),(0,0,255),2)

    #cv2.drawContours(imgPerspective, contours, 2, (0, 255, 0), 2)
    approx=cv2.approxPolyDP(contours_right, 0.01*cv2.arcLength(contours_right,True),True)
    x,y,w,h=cv2.boundingRect(approx)

    cv2.rectangle(imgPerspective,(x,y),(x+w,y+h),(0,0,255),2)
    """
    for i in range(len(contours)):
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        print(area)
        if(area > 100):
            cv2.drawContours(imgPerspective, contours, i, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)

            cv2.rectangle(imgPerspective,(x,y),(x+w,y+h),(0,0,255),2)"""

    cv2.imshow("Camera", imgPerspective)
    if cv2.waitKey(1)==ord("q"):#wait until key ‘q’ pressed
        break
cv2.destroyAllWindows()
