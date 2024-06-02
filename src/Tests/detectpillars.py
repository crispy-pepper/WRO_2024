#libraries
import sys
import cv2
import numpy as np
import time
sys.path.append('/home/pi/TurboPi/')
from picamera2 import Picamera2
import HiwonderSDK.Board as Board

ROI_LEFT = [0, 0, 300, 480]
ROI_RIGHT = [340, 0, 640, 480]
WIDTH = 640
HEIGHT = 480


LOWER_RED_THRESHOLD = np.array([119, 120, 27])#119,120,27
UPPER_RED_THRESHOLD = np.array([180, 255, 255])

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 55])

LOWER_GREEN_THRESHOLD = np.array([35, 100, 40])
UPPER_GREEN_THRESHOLD = np.array([110, 255, 160])


ACTIONS_TO_STRAIGHT = 245
WALL_THRESHOLD = 300
NO_WALL_THRESHOLD = 600


# camera setup
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 35
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# utility function for pwm angle calculation
def pwm(degree):
	return round(degree*11.1 + 500)


time.sleep(2)
print("---------------------------- running--------------------------")



while True:
    # setup camera frame
    im= picam2.capture_array()
    max_green_area = 0
    max_red_area = 0

    # convert to hsv
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    
    # set default DC motor speed as straight section speed
    image = cv2.line(im, (ROI_LEFT[0], ROI_LEFT[1]), (ROI_LEFT[2], ROI_LEFT[1]), (0, 255, 255), 2)
    image = cv2.line(im, (ROI_LEFT[0], ROI_LEFT[1]), (ROI_LEFT[0], ROI_LEFT[3]), (0, 255, 255), 2)
    image = cv2.line(im, (ROI_LEFT[2], ROI_LEFT[3]), (ROI_LEFT[2], ROI_LEFT[1]), (0, 255, 255), 2)
    image = cv2.line(im, (ROI_LEFT[2], ROI_LEFT[3]), (ROI_LEFT[0], ROI_LEFT[3]), (0, 255, 255), 2)
    
    image = cv2.line(im, (ROI_RIGHT[0], ROI_RIGHT[1]), (ROI_RIGHT[2], ROI_RIGHT[1]), (0, 255, 255), 2)
    image = cv2.line(im, (ROI_RIGHT[0], ROI_RIGHT[1]), (ROI_RIGHT[0], ROI_RIGHT[3]), (0, 255, 255), 2)
    image = cv2.line(im, (ROI_RIGHT[2], ROI_RIGHT[3]), (ROI_RIGHT[2], ROI_RIGHT[1]), (0, 255, 255), 2)
    image = cv2.line(im, (ROI_RIGHT[2], ROI_RIGHT[3]), (ROI_RIGHT[0], ROI_RIGHT[3]), (0, 255, 255), 2)
        
    # find black thresholds

    img_thresh_black = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)
    img_thresh_red = cv2.inRange(img_hsv, LOWER_RED_THRESHOLD, UPPER_RED_THRESHOLD)
    img_thresh_green = cv2.inRange(img_hsv, LOWER_GREEN_THRESHOLD, UPPER_GREEN_THRESHOLD)
    
    # define functions for right and left contours in respective ROIs
    left_contours_red, hierarchy = cv2.findContours(img_thresh_red[ROI_LEFT[1]:ROI_LEFT[3], ROI_LEFT[0]:ROI_LEFT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    right_contours_red, hierarchy = cv2.findContours(img_thresh_red[ROI_RIGHT[1]:ROI_RIGHT[3], ROI_RIGHT[0]:ROI_RIGHT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    left_contours_green, hierarchy = cv2.findContours(img_thresh_green[ROI_LEFT[1]:ROI_LEFT[3], ROI_LEFT[0]:ROI_LEFT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    right_contours_green, hierarchy = cv2.findContours(img_thresh_green[ROI_RIGHT[1]:ROI_RIGHT[3], ROI_RIGHT[0]:ROI_RIGHT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # find all contours for debug utility
    contours_black, hierarchy = cv2.findContours(img_thresh_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
        
    
    #contours = [(contours_black,(255,255,255),2),(left_contours_red,(0,0,255),1),(right_contours_red,(0,0,255),0),(left_contours_green,(0,255,0),1),(right_contours_green,(0,255,0),0)]
    contours = [(left_contours_red,(0,0,255),1),(right_contours_red,(0,0,255),0),(left_contours_green,(0,255,0),1),(right_contours_green,(0,255,0),0)]

    for c in contours:
        cont = c[0]
        colour = c[1]
        for i,cnt in enumerate(cont):

            area = cv2.contourArea(cnt)
            if area > 100:
                cv2.drawContours(im, cont, i, colour,1)
                approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
                x,y,w,h=cv2.boundingRect(approx)
            if c[2]!= 2 and area > 2000:
                Board.RGB.setPixelColor(c[2], Board.PixelColor(colour[2],colour[1],colour[0]))
            else:
                Board.RGB.setPixelColor(c[2], Board.PixelColor(0,0,0))

            Board.RGB.show()

    
    
    # display the camera
    cv2.imshow("Camera", im)
    
    
    # if the number of actions to the straight section has been met, stop the car
    if (cv2.waitKey(1)==ord("q")):#
        time.sleep(0.02)
        print("stop")
        
        break
    
    
cv2.destroyAllWindows()
