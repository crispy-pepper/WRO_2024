#libraries
import sys
import cv2
import numpy as np
import time
sys.path.append('/home/pi/TurboPi/')
from picamera2 import Picamera2
import HiwonderSDK.Board as Board

# constant variables
MID_SERVO = 80
MAX_TURN_DEGREE = 50


ROI_LEFT_BOT = [0, 290, 100, 330]
ROI_RIGHT_BOT = [540, 290, 640, 330]

ROI_LEFT_TOP = [0, 270, 50, 290]
ROI_RIGHT_TOP = [590, 270, 640, 290]

ROI_PILLAR = [50,50,430,430]

PD = 0.0036
PG = 0.009
WIDTH = 640
HEIGHT = 480
POINTS = [(115,200), (525,200), (640,370), (0,370)]
LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 70])
DC_STRAIGHT_SPEED = 1350
DC_TURN_SPEED = 1364
MAX_TURNS = 12
ACTIONS_TO_STRAIGHT = 400
WALL_THRESHOLD = 600
NO_WALL_THRESHOLD = 50

LOWER_RED_THRESHOLD1 = np.array([0, 50, 50])
UPPER_RED_THRESHOLD1 = np.array([10, 255, 255])
LOWER_RED_THRESHOLD2 = np.array([170, 50, 50])
UPPER_RED_THRESHOLD2 = np.array([180, 255, 255])

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 55])

LOWER_GREEN_THRESHOLD = np.array([60, 125, 80])
UPPER_GREEN_THRESHOLD = np.array([90, 255, 155])


#dynamic variables
sharp_turn_left = False
sharp_turn_right = False
total_turn = 0
action_counter = 0
last_difference = 0
current_difference = 0
servo_angle=0 

turning_iter = 0

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

    

Board.setPWMServoPulse(1, pwm(MID_SERVO), 10) #turn servo to mid
Board.setPWMServoPulse(6, 1500, 100) # arm the esc motor
time.sleep(2)
print("---------------------------- running--------------------------")



while True:
    # setup camera frame
    im= picam2.capture_array()

    # convert to hsv
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        
     # find black thresholds
    img_thresh = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)


    #left_contours = np.concatenate((left_contours_top,left_contours_bot))
    #right_contours =  np.concatenate((right_contours_top,right_contours_bot))
    
    # find all contours for debug utility
    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    img_thresh_black = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)
    img_thresh_red1 = cv2.inRange(img_hsv, LOWER_RED_THRESHOLD1, UPPER_RED_THRESHOLD1)
    img_thresh_red2 = cv2.inRange(img_hsv, LOWER_RED_THRESHOLD2, UPPER_RED_THRESHOLD2)
    img_thresh_green = cv2.inRange(img_hsv, LOWER_GREEN_THRESHOLD, UPPER_GREEN_THRESHOLD)
    
    img_thresh_red = img_thresh_red2|img_thresh_red1
    # define functions for right and left contours in respective ROIs
    
    left_contours_top, hierarchy = cv2.findContours(img_thresh_black[ROI_LEFT_TOP[1]:ROI_LEFT_TOP[3], ROI_LEFT_TOP[0]:ROI_LEFT_TOP[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    right_contours_top, hierarchy = cv2.findContours(img_thresh_black[ROI_RIGHT_TOP[1]:ROI_RIGHT_TOP[3], ROI_RIGHT_TOP[0]:ROI_RIGHT_TOP[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    left_contours_bot, hierarchy = cv2.findContours(img_thresh_black[ROI_LEFT_BOT[1]:ROI_LEFT_BOT[3], ROI_LEFT_BOT[0]:ROI_LEFT_BOT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    right_contours_bot, hierarchy = cv2.findContours(img_thresh_black[ROI_RIGHT_BOT[1]:ROI_RIGHT_BOT[3], ROI_RIGHT_BOT[0]:ROI_RIGHT_BOT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # find all contours for debug utility
    contours_black, hierarchy = cv2.findContours(img_thresh_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    
    contour_red, hierarchy = cv2.findContours(img_thresh_red[ROI_PILLAR[1]:ROI_PILLAR[3], ROI_PILLAR[0]:ROI_PILLAR[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contour_green, hierarchy = cv2.findContours(img_thresh_green[ROI_PILLAR[1]:ROI_PILLAR[3], ROI_PILLAR[0]:ROI_PILLAR[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
    
    #contours = [(contours_black,(255,255,255),2),(left_contours_red,(0,0,255),1),(right_contours_red,(0,0,255),0),(left_contours_green,(0,255,0),1),(right_contours_green,(0,255,0),0)]
    contours = [(contour_red,(0,0,255),1),(contour_green,(0,255,0),0),(contours_black,(0,0,0),2)]

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

    
    image = cv2.line(im, (ROI_LEFT_TOP[0], ROI_LEFT_TOP[1]), (ROI_LEFT_TOP[2], ROI_LEFT_TOP[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_LEFT_TOP[0], ROI_LEFT_TOP[1]), (ROI_LEFT_TOP[0], ROI_LEFT_TOP[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_LEFT_TOP[2], ROI_LEFT_TOP[3]), (ROI_LEFT_TOP[2], ROI_LEFT_TOP[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_LEFT_TOP[2], ROI_LEFT_TOP[3]), (ROI_LEFT_TOP[0], ROI_LEFT_TOP[3]), (0, 255, 255), 4)
    
    image = cv2.line(im, (ROI_RIGHT_TOP[0], ROI_RIGHT_TOP[1]), (ROI_RIGHT_TOP[2], ROI_RIGHT_TOP[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_RIGHT_TOP[0], ROI_RIGHT_TOP[1]), (ROI_RIGHT_TOP[0], ROI_RIGHT_TOP[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_RIGHT_TOP[2], ROI_RIGHT_TOP[3]), (ROI_RIGHT_TOP[2], ROI_RIGHT_TOP[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_RIGHT_TOP[2], ROI_RIGHT_TOP[3]), (ROI_RIGHT_TOP[0], ROI_RIGHT_TOP[3]), (0, 255, 255), 4)   


    image = cv2.line(im, (ROI_LEFT_BOT[0], ROI_LEFT_BOT[1]), (ROI_LEFT_BOT[2], ROI_LEFT_BOT[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_LEFT_BOT[0], ROI_LEFT_BOT[1]), (ROI_LEFT_BOT[0], ROI_LEFT_BOT[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_LEFT_BOT[2], ROI_LEFT_BOT[3]), (ROI_LEFT_BOT[2], ROI_LEFT_BOT[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_LEFT_BOT[2], ROI_LEFT_BOT[3]), (ROI_LEFT_BOT[0], ROI_LEFT_BOT[3]), (0, 255, 255), 4)
    
    image = cv2.line(im, (ROI_RIGHT_BOT[0], ROI_RIGHT_BOT[1]), (ROI_RIGHT_BOT[2], ROI_RIGHT_BOT[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_RIGHT_BOT[0], ROI_RIGHT_BOT[1]), (ROI_RIGHT_BOT[0], ROI_RIGHT_BOT[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_RIGHT_BOT[2], ROI_RIGHT_BOT[3]), (ROI_RIGHT_BOT[2], ROI_RIGHT_BOT[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_RIGHT_BOT[2], ROI_RIGHT_BOT[3]), (ROI_RIGHT_BOT[0], ROI_RIGHT_BOT[3]), (0, 255, 255), 4)   

    image = cv2.line(im, (ROI_PILLAR[0], ROI_PILLAR[1]), (ROI_PILLAR[2], ROI_PILLAR[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_PILLAR[0], ROI_PILLAR[1]), (ROI_PILLAR[0], ROI_PILLAR[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_PILLAR[2], ROI_PILLAR[3]), (ROI_PILLAR[2], ROI_PILLAR[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_PILLAR[2], ROI_PILLAR[3]), (ROI_PILLAR[0], ROI_PILLAR[3]), (0, 255, 255), 4)
    

    cv2.imshow("Camera", im)
    
    # if the number of actions to the straight section has been met, stop the car
    if (cv2.waitKey(1)==ord("q") or action_counter >= ACTIONS_TO_STRAIGHT):#
        time.sleep(0.02)
        Board.setPWMServoPulse(6, 1500, 100) 
        Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
        print("stop")
        
        break
    
    
cv2.destroyAllWindows()
