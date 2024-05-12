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
MAX_TURN_DEGREE = 32
ROI_LEFT = [0, 300, 150, 335]
ROI_RIGHT = [490, 300, 640, 335]
PD = 0.0036
PG = 0.008
WIDTH = 640
HEIGHT = 480
POINTS = [(115,200), (525,200), (640,370), (0,370)]
LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 55])
DC_STRAIGHT_SPEED = 1342
DC_TURN_SPEED = 1346
MAX_TURNS = 12
ACTIONS_TO_STRAIGHT = 245
WALL_THRESHOLD = 300
NO_WALL_THRESHOLD = 600

#dynamic variables
sharp_turn_left = False
sharp_turn_right = False
total_turn = 0
action_counter = 0
last_difference = 0
current_difference = 0


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
    input = np.float32(POINTS)
    output = np.float32([(0,0), (WIDTH-1,0), (WIDTH-1,HEIGHT-1), (0,HEIGHT-1)])

    # convert to hsv
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        
     # find black thresholds
    img_thresh = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)
    
    # define functions for right and left contours in respective ROIs
    left_contours, hierarchy = cv2.findContours(img_thresh[ROI_LEFT[1]:ROI_LEFT[3], ROI_LEFT[0]:ROI_LEFT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    right_contours, hierarchy = cv2.findContours(img_thresh[ROI_RIGHT[1]:ROI_RIGHT[3], ROI_RIGHT[0]:ROI_RIGHT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # find all contours for debug utility
    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # define variables for left and right ROI contour areas
    left_area = 0
    right_area = 0
    
    # loop to find largest contours
    for i in range(len(left_contours)):
        cnt = left_contours[i]
        area = cv2.contourArea(cnt)
        left_area = max(area, left_area)
        
        
    
    for i in range(len(right_contours)):
        cnt = right_contours[i]
        area = cv2.contourArea(cnt)
        
        right_area = max(area, right_area)
        
        
    # draw all relatively large contours for debug utility
    for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            if area >100:
                cv2.drawContours(im, contours, i, (0, 255, 0), 2)
                approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
                x,y,w,h=cv2.boundingRect(approx)
        
    
    # set default DC motor speed as straight section speed
    
    dc_speed = DC_STRAIGHT_SPEED
    
    
    if (sharp_turn_right and right_area< WALL_THRESHOLD):
            servo_angle = MID_SERVO-MAX_TURN_DEGREE
            dc_speed = DC_TURN_SPEED
            
    elif (sharp_turn_left and left_area< WALL_THRESHOLD):
            servo_angle = MID_SERVO+MAX_TURN_DEGREE
            dc_speed = DC_TURN_SPEED
    
    
    
    
    
    else:
        sharp_turn_left = False
        sharp_turn_right = False
        
        if right_area < NO_WALL_THRESHOLD:
            print("no wall to the right") 
            
            # set all movement variables to turn sharply right
            
            
            sharp_turn_right = True
            
            
            total_turn+=1
            
            print(str(total_turn) + "th turn") 
            
        elif left_area < NO_WALL_THRESHOLD:
            print("no wall to the left")
            
            # set all movement variables to turn sharply left
            
            sharp_turn_left = True
            
          
            total_turn+=1
            
            
            print(str(total_turn) + "th turn")


        else:
            # if in the straight section, calculate the current_difference between the contours in the left and right area
            current_difference = left_area - right_area
            print ("current current_difference: " + str(current_difference))
            if (left_area > right_area):
                print ("left bigger")
            else:
                print ("right bigger")
            #calculate steering amount using preportional-derivative steering
            # multiply the current_difference by a constant variable and add the projected error multiplied by another constand
            servo_angle = MID_SERVO - (current_difference * PG + (current_difference-last_difference) * PD)
            print (MID_SERVO - (current_difference * PG + (current_difference-last_difference) * PD))
                


#   #if the total turns has surpassed the amount required, increment the action counter by 1
    if total_turn == MAX_TURNS:
        action_counter += 1
        
    # set the last current_difference equal to the current current_difference for derivative steering
    last_difference= current_difference
    
    # if the steering variable is higher than the max turn degree for the servo, set it to the max turn degree
    if (servo_angle < MID_SERVO - MAX_TURN_DEGREE):
        servo_angle = MID_SERVO - MAX_TURN_DEGREE
        
    elif (servo_angle > MID_SERVO + MAX_TURN_DEGREE):
        servo_angle = MID_SERVO + MAX_TURN_DEGREE
       
        
    print ("turning " + str(servo_angle))
    
    
    # move the motors using the variables
    pw = pwm(servo_angle)
    Board.setPWMServoPulse(6, dc_speed, 100) 
    Board.setPWMServoPulse(1, pw, 1000)
      
               
    #draw the ROIs
    image = cv2.line(im, (ROI_LEFT[0], ROI_LEFT[1]), (ROI_LEFT[2], ROI_LEFT[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_LEFT[0], ROI_LEFT[1]), (ROI_LEFT[0], ROI_LEFT[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_LEFT[2], ROI_LEFT[3]), (ROI_LEFT[2], ROI_LEFT[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_LEFT[2], ROI_LEFT[3]), (ROI_LEFT[0], ROI_LEFT[3]), (0, 255, 255), 4)
    
    image = cv2.line(im, (ROI_RIGHT[0], ROI_RIGHT[1]), (ROI_RIGHT[2], ROI_RIGHT[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_RIGHT[0], ROI_RIGHT[1]), (ROI_RIGHT[0], ROI_RIGHT[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_RIGHT[2], ROI_RIGHT[3]), (ROI_RIGHT[2], ROI_RIGHT[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI_RIGHT[2], ROI_RIGHT[3]), (ROI_RIGHT[0], ROI_RIGHT[3]), (0, 255, 255), 4)   
    
    
    # display the camera
    cv2.imshow("Camera", im)
    
    
    # if the number of actions to the straight section has been met, stop the car
    if (cv2.waitKey(1)==ord("q") or action_counter >= ACTIONS_TO_STRAIGHT):#
        time.sleep(0.02)
        Board.setPWMServoPulse(6, 1500, 100) 
        Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
        print("stop")
        
        break
    
    
cv2.destroyAllWindows()
