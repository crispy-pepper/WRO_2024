import sys

import cv2
import numpy as np
import time
sys.path.append('/home/pi/TurboPi/')
from picamera2 import Picamera2
import HiwonderSDK.Board as Board
# used to record the time when we processed last frame
prev_frame_time = 0
mid = 80
max_turn_degree = 32

# used to record the time at which we processed current frame
new_frame_time = 0
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
def pwm(degree):
	return round(degree*11.1 + 500)
ROI1 = [20, 310, 240, 350]
ROI2 = [400, 310, 620, 350]
PG = 0.000001
points = [(115,200), (525,200), (640,370), (0,370)]

Board.setPWMServoPulse(1, pwm(mid), 10) #Turn to 90 degree
# 'Arm' the ESC
Board.setPWMServoPulse(6, 1500, 100)

while True:
    s = 0
    im= picam2.capture_array()
    width = 640
    height = 480
    input = np.float32(points)
    output = np.float32([(0,0), (width-1,0), (width-1,height-1), (0,height-1)])
    # compute perspective matrix
    
  
    #grayscaling
    #thresholding
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        
        # black mask
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([220, 255, 65])
    
    lower_blue = np.array([50, 50, 50])
    upper_blue = np.array([180, 180, 255])
        
    imgThresh = cv2.inRange(img_hsv, lower_black, upper_black)
    
    
    leftContours, hierarchy = cv2.findContours(imgThresh[ROI1[1]:ROI1[3], ROI1[0]:ROI1[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    rightContours, hierarchy = cv2.findContours(imgThresh[ROI2[1]:ROI2[3], ROI2[0]:ROI2[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours, hierarchy = cv2.findContours(imgThresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    leftArea = 0
    rightArea = 0
    for i in range(len(leftContours)):
        cnt = leftContours[i]
        area = cv2.contourArea(cnt)
        if (area > 100):
            print(area)
        leftArea = max(area, leftArea)
        
        
            
    for i in range(len(rightContours)):
        cnt = rightContours[i]
        area = cv2.contourArea(cnt)
        
        #if (area > 100):
            #print(area)
        
        rightArea = max(area, rightArea)
        
    for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            if area >100:
                cv2.drawContours(im, contours, i, (0, 255, 0), 2)
                approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
                x,y,w,h=cv2.boundingRect(approx)
        
        
  
            
        
    if rightArea < 300:
        print("no wall to the right")
        s = mid-max_turn_degree - 10
    elif leftArea < 300:
        print("no wall to the left")
        s = mid+max_turn_degree -10
    else:
        difference = leftArea - rightArea
        print ("current difference: " + str(difference))
        if (leftArea > rightArea):
            print ("left bigger")
            s = (mid-max_turn_degree) * difference * PG

            
        else:
            print ("right bigger")
            s = (mid+max_turn_degree) * difference * PG

    b = 1340
    image = cv2.line(im, (ROI1[0], ROI1[1]), (ROI1[2], ROI1[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI1[0], ROI1[1]), (ROI1[0], ROI1[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI1[2], ROI1[3]), (ROI1[2], ROI1[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI1[2], ROI1[3]), (ROI1[0], ROI1[3]), (0, 255, 255), 4)
    
    image = cv2.line(im, (ROI2[0], ROI2[1]), (ROI2[2], ROI2[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI2[0], ROI2[1]), (ROI2[0], ROI2[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI2[2], ROI2[3]), (ROI2[2], ROI2[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI2[2], ROI2[3]), (ROI2[0], ROI2[3]), (0, 255, 255), 4)   
    cv2.imshow("Camera", im)
    pw = pwm(s)
    Board.setPWMServoPulse(6, b, 100) 
    Board.setPWMServoPulse(1, pw, 1000)
    if cv2.waitKey(1)==ord("q"):#wait until key ‘q’ pressed
        time.sleep(0.02)
        Board.setPWMServoPulse(6, 1500, 100) 
        Board.setPWMServoPulse(1, pwm(mid), 1000)
        print("stop")
        
        break
    
    
cv2.destroyAllWindows()
