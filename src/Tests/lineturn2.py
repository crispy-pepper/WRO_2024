
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
ROI_LEFT_BOT = [0, 300, 100, 340] 
ROI_RIGHT_BOT = [540, 300, 640, 340]
ROI_LEFT_TOP = [0, 280, 50, 300]
ROI_RIGHT_TOP = [590, 280, 640, 300]
ROI4 = [300, 340, 340, 380]


turnDir = "none"
PD = 0.0065
PG = 0.0065
WIDTH = 640
HEIGHT = 480
POINTS = [(115,200), (525,200), (640,370), (0,370)]
LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 64])
DC_STRAIGHT_SPEED = 1345
DC_TURN_SPEED = 1345
MAX_TURNS = 12
ACTIONS_TO_STRAIGHT = 200
WALL_THRESHOLD = 50
NO_WALL_THRESHOLD = 50
TURN_ITER_LIMIT = 160
LINE_THRESHOLD = 35

LOWER_RED_THRESHOLD1 = np.array([0, 50, 50])
UPPER_RED_THRESHOLD1 = np.array([10, 255, 255])
LOWER_RED_THRESHOLD2 = np.array([167, 50, 50])
UPPER_RED_THRESHOLD2 = np.array([180, 255, 255])


PILLAR_SIZE = 3200


#LOWER_GREEN_THRESHOLD = np.array([58, 42, 60])
#UPPER_GREEN_THRESHOLD = np.array([116, 255, 255])

LOWER_GREEN_THRESHOLD = np.array([58, 62, 55])
UPPER_GREEN_THRESHOLD = np.array([96, 255, 255])

#dynamic variables
sharp_turn_left = False
sharp_turn_right = False
total_turn = 0
action_counter = 0
last_difference = 0
current_difference = 0
servo_angle=0 
lower_blue = np.array([100, 49, 47])
upper_blue = np.array([140, 122, 130])
trackDir = "none"
turncounter = 0
        
        
lower_orange1 = np.array([155, 57, 70])
upper_orange1 = np.array([180, 255, 255])
lower_orange2 = np.array([0, 57, 70])
upper_orange2 = np.array([8, 255, 255])

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
    input = np.float32(POINTS)
    output = np.float32([(0,0), (WIDTH-1,0), (WIDTH-1,HEIGHT-1), (0,HEIGHT-1)])

    # convert to hsv
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        
     # find black thresholds
    img_thresh = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)
    
    # define functions for right and left contours in respective ROIs
    left_contours_top, hierarchy = cv2.findContours(img_thresh[ROI_LEFT_TOP[1]:ROI_LEFT_TOP[3], ROI_LEFT_TOP[0]:ROI_LEFT_TOP[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    right_contours_top, hierarchy = cv2.findContours(img_thresh[ROI_RIGHT_TOP[1]:ROI_RIGHT_TOP[3], ROI_RIGHT_TOP[0]:ROI_RIGHT_TOP[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    left_contours_bot, hierarchy = cv2.findContours(img_thresh[ROI_LEFT_BOT[1]:ROI_LEFT_BOT[3], ROI_LEFT_BOT[0]:ROI_LEFT_BOT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    right_contours_bot, hierarchy = cv2.findContours(img_thresh[ROI_RIGHT_BOT[1]:ROI_RIGHT_BOT[3], ROI_RIGHT_BOT[0]:ROI_RIGHT_BOT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    #left_contours = np.concatenate((left_contours_top,left_contours_bot))
    #right_contours =  np.concatenate((right_contours_top,right_contours_bot))
    
    # find all contours for debug utility
    contours, hierarchy = cv2.findContours(img_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # define variables for left and right ROI contour areas
    left_area_top = 0
    left_area_bot = 0

    right_area_top = 0
    right_area_bot = 0
    
    # loop to find largest contours
    for i in range(len(left_contours_top)):
        cnt = left_contours_top[i]
        area = cv2.contourArea(cnt)
        left_area_top = max(area, left_area_top)
    
    for i in range(len(left_contours_bot)):
        cnt = left_contours_bot[i]
        area = cv2.contourArea(cnt)
        left_area_bot = max(area, left_area_bot)
        
        
    
    for i in range(len(right_contours_top)):
        cnt = right_contours_top[i]

        area = cv2.contourArea(cnt)
        
        right_area_top = max(area, right_area_top)
    
    for i in range(len(right_contours_bot)):
        cnt = right_contours_bot[i]

        area = cv2.contourArea(cnt)
        
        right_area_bot = max(area, right_area_bot)
        
    right_area = right_area_bot+right_area_top
    left_area = left_area_bot+left_area_top
    # draw all relatively large contours for debug utility
    for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            if area >100:
                cv2.drawContours(im, contours, i, (0, 255, 0), 2)
                approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
                x,y,w,h=cv2.boundingRect(approx)
        
    
    b_mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

    #find blue contours to detect the lines on the mat
    contours_blue = cv2.findContours(b_mask[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    o_mask = cv2.bitwise_or(cv2.inRange(img_hsv, lower_orange1, upper_orange1), cv2.inRange(img_hsv, lower_orange2, upper_orange2))

    #find orange contours to detect the lines on the mat
    contours_orange = cv2.findContours(o_mask[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
        
    max_blue_area = 0
    max_orange_area = 0
    for i in range(len(contours_orange)):
        cnt = contours_orange[i]
        max_orange_area = max(cv2.contourArea(cnt), max_orange_area)
        cnt[:, :, 0] += ROI4[0]  # Add X offset
        cnt[:, :, 1] += ROI4[1]  # Add Y offset
             
        cv2.drawContours(im, contours_orange, i, (255, 255, 0), 1)
        

        #iterate through blue contours
    for i in range(len(contours_blue)):
        cnt = contours_blue[i]
        max_blue_area = max(cv2.contourArea(cnt), max_blue_area)
        cnt[:, :, 0] += ROI4[0]  # Add X offset
        cnt[:, :, 1] += ROI4[1]  # Add Y offset
        #if the turn direction is left
        cv2.drawContours(im, contours_blue, i, (255, 255, 0), 1)
        

    if trackDir == "none":
        if max_blue_area>max_orange_area and max_blue_area > LINE_THRESHOLD:
            trackDir = "left"
            print("I see more blue than orange and change track dir")
        if max_orange_area > max_blue_area and max_orange_area > LINE_THRESHOLD:
            trackDir = "right"
            print("I see more orange than blue and change track dir")
            

    
    
    if trackDir == "right":
        if (turnDir == "right" and max_blue_area > LINE_THRESHOLD and max_orange_area < LINE_THRESHOLD):   
                
            turnDir = "none"
            print("done turning")
                
        elif max_orange_area > LINE_THRESHOLD:

                #if the turn direction hasn't been changed yet change the turn direction to right
            if turnDir == "none":
                turncounter += 1
                turnDir = "right"    
                print(turnDir,turncounter)

        
                

    elif trackDir == "left":
        if (turnDir == "left" and max_orange_area > LINE_THRESHOLD and max_blue_area < LINE_THRESHOLD):
                

            turnDir = "none"
            print("done turning")
                
        elif max_blue_area > LINE_THRESHOLD:
                #if the turn direction hasn't been changed yet change the turn direction to left
            if turnDir == "none":
                turncounter += 1
                turnDir = "left" 
                print(turnDir,turncounter)
            
    
    # set default DC motor speed as straight section speed
    
    dc_speed = DC_STRAIGHT_SPEED
    
        
    

    if turnDir == 'right':
        servo_angle = MID_SERVO-MAX_TURN_DEGREE 
        dc_speed = DC_TURN_SPEED
        
            
    elif turnDir == 'left':

        servo_angle = MID_SERVO+MAX_TURN_DEGREE 
        dc_speed = DC_TURN_SPEED
    
    
    
    
    
    else:
        turnDir = "none"
       
        turning_iter = 0
        
        if left_area < WALL_THRESHOLD:
            servo_angle = MID_SERVO+MAX_TURN_DEGREE
        elif right_area < WALL_THRESHOLD:
            servo_angle = MID_SERVO+MAX_TURN_DEGREE 
 

        else:
            # if in the straight section, calculate the current_difference between the contours in the left and right area
            current_difference = left_area - right_area
            #print ("current current_difference: " + str(current_difference))
            if (left_area > right_area):
                #print ("left bigger")
                pass
            else:
                #print ("right bigger")
                pass
            #calculate steering amount using preportional-derivative steering
            # multiply the current_difference by a constant variable and add the projected error multiplied by another constand
            servo_angle = MID_SERVO - (current_difference * PG + (current_difference-last_difference) * PD)
            #print (MID_SERVO - (current_difference * PG + (current_difference-last_difference) * PD))
            print("PG: " + str(current_difference * PG))
            print("PD: " + str((current_difference-last_difference) * PD))
            print(servo_angle)
        #   #if the total turns has surpassed the amount required, increment the action counter by 1

    if turncounter == MAX_TURNS:
        action_counter += 1
    

        
            


    
        
    # set the last current_difference equal to the current current_difference for derivative steering
    last_difference= current_difference
    
    # if the steering variable is higher than the max turn degree for the servo, set it to the max turn degree
    if (servo_angle < MID_SERVO - MAX_TURN_DEGREE):
        servo_angle = MID_SERVO - MAX_TURN_DEGREE
        
    elif (servo_angle > MID_SERVO + MAX_TURN_DEGREE):
        servo_angle = MID_SERVO + MAX_TURN_DEGREE
       
        
    #print ("turning " + str(servo_angle))
    
    
    # move the motors using the variables
    pw = pwm(servo_angle)
    Board.setPWMServoPulse(6, dc_speed, 100) 
    Board.setPWMServoPulse(1, pw, 1000)
      
               
    #draw the ROIs
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
    
    image = cv2.line(im, (ROI4[0], ROI4[1]), (ROI4[2], ROI4[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI4[0], ROI4[1]), (ROI4[0], ROI4[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI4[2], ROI4[3]), (ROI4[2], ROI4[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI4[2], ROI4[3]), (ROI4[0], ROI4[3]), (0, 255, 255), 4)      
    
    
    # display the camera
    cv2.imshow("Camera", im)
    
    
    # if the number of actions to the straight section has been met, stop the car
    if (cv2.waitKey(1)==ord("q") or action_counter >= ACTIONS_TO_STRAIGHT):#
        Board.setPWMServoPulse(6, 1500, 100) 
        time.sleep(0.02)
        Board.setPWMServoPulse(6, 1500, 100) 
        Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
        print("stop")
        
        break
    
    
cv2.destroyAllWindows()

