
#libraries
import math
import sys
import cv2
import numpy as np
import time
sys.path.append('/home/pi/TurboPi/')
from picamera2 import Picamera2
import HiwonderSDK.Board as Board
from libcamera import controls
# constant variables
MID_SERVO = 82
MAX_TURN_DEGREE = 38
ROI_LEFT_BOT = [0, 300, 100, 340]
ROI_RIGHT_BOT = [540, 300, 640, 340]
ROI_LEFT_TOP = [0, 270, 50, 300]
ROI_RIGHT_TOP = [590, 270, 640, 300]
ROI4 = [270, 310, 360, 340]
ROI_MIDDLE = [0, 200, 640, 380]


PD = 0.12
PG = 0.0028
RED_TARGET = 160
GREEN_TARGET = 480
WIDTH = 640
HEIGHT = 480
POINTS = [(115,200), (525,200), (640,370), (0,370)]
LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])

UPPER_BLACK_THRESHOLD = np.array([180, 255, 80])
#UPPER_BLACK_THRESHOLD = np.array([180, 255, 20])
LOWER_RED_THRESHOLD1 = np.array([0, 160, 80])
UPPER_RED_THRESHOLD1 = np.array([4, 255, 185])
LOWER_RED_THRESHOLD2 = np.array([150, 160, 80])
UPPER_RED_THRESHOLD2 = np.array([180, 255, 255])
#196 for value in lighter conditions
#LOWER_RED_THRESHOLD1 = np.array([0, 150, 110])
#UPPER_RED_THRESHOLD1 = np.array([0, 255, 215])
#LOWER_RED_THRESHOLD2 = np.array([150, 152, 40])
#UPPER_RED_THRESHOLD2 = np.array([164, 225, 215])

LOWER_GREEN_THRESHOLD = np.array([50, 20, 50])
UPPER_GREEN_THRESHOLD = np.array([107, 255, 215])
#LOWER_GREEN_THRESHOLD = np.array([43, 110, 60])
#UPPER_GREEN_THRESHOLD = np.array([106, 245, 185])
LOWER_ORANGE_THRESHOLD1 = np.array([164, 100, 150])
UPPER_ORANGE_THRESHOLD1 = np.array([180, 215, 255])
LOWER_ORANGE_THRESHOLD2 = np.array([0, 100, 196])
UPPER_ORANGE_THRESHOLD2 = np.array([20, 255, 255])

#LOWER_ORANGE_THRESHOLD1 = np.array([155, 130, 72])
#UPPER_ORANGE_THRESHOLD1 = np.array([180, 255, 125])
#LOWER_ORANGE_THRESHOLD2 = np.array([0, 80, 90])
#UPPER_ORANGE_THRESHOLD2 = np.array([0, 255, 195])
LOWER_BLUE_THRESHOLD = np.array([110, 73,60])
UPPER_BLUE_THRESHOLD = np.array([132, 205, 236])    
#LOWER_BLUE_THRESHOLD = np.array([104, 73,40])
#UPPER_BLUE_THRESHOLD = np.array([132, 205, 115])    

LOWER_MAGENTA_THRESHOLD= np.array([168, 175, 50])
UPPER_MAGENTA_THRESHOLD = np.array([172, 255, 255])
LINE_THRESHOLD =35
PILLAR_SIZE =500
DC_STRAIGHT_SPEED = 1343
DC_TURN_SPEED = 1343
MAX_TURNS = 12
ACTIONS_TO_STRAIGHT = 400

#dynamic variables
last_target = None
dc_speed = DC_STRAIGHT_SPEED
num_pillars_g = 0
num_pillars_r = 0
error = 0
sharp_turn_left = False
sharp_turn_right = False
total_turn = 0
action_counter = 0
last_difference = 0
current_difference = 0
servo_angle=0 
lastLapTurnAround = False
prevPillar = ""
prevError = 0
turning_iter = 0
turnDir = "none"
trackDir = "none"
lastLapDone = False
target = 'none'
# camera setup
picam2 = Picamera2()
#picam2.set_controls({"Brightness" : 1.0})
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 25
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.set_controls({ "Saturation":1.4, "Brightness" : 0.05})

picam2.start()
# utility function for pwm angle calculation
def pwm(degree):
	return round(degree*11.1 + 500)

def stop():
    time.sleep(0.02)
    Board.setPWMServoPulse(6, 1500, 100) 
    Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
    time.sleep(0.05)
    Board.setPWMServoPulse(6, 1500, 100) 


    

Board.setPWMServoPulse(1, pwm(MID_SERVO), 1) #turn servo to mid
Board.setPWMServoPulse(6, 1500, 100) # arm the esc motor
time.sleep(2)
print("---------------------------- running--------------------------")

while True:
    OBSTACLEPG = 0.0017
    OBSTACLEPD = 0 
    YAXISPG = 0.05
    # setup camera frame
    im = picam2.capture_array()
    input = np.float32(POINTS)
    output = np.float32([(0,0), (WIDTH-1,0), (WIDTH-1,HEIGHT-1), (0,HEIGHT-1)])

    # convert to hsv
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        
    # find black thresholds
    img_thresh = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)
    img_thresh_red1 = cv2.inRange(img_hsv, LOWER_RED_THRESHOLD1, UPPER_RED_THRESHOLD1)
    img_thresh_red2 = cv2.inRange(img_hsv, LOWER_RED_THRESHOLD2, UPPER_RED_THRESHOLD2)
    img_thresh_green = cv2.inRange(img_hsv, LOWER_GREEN_THRESHOLD, UPPER_GREEN_THRESHOLD)
    img_thresh_red = cv2.bitwise_or(img_thresh_red1, img_thresh_red2)
    #img_thresh_red = cv2.inRange(img_hsv, LOWER_RED_THRESHOLD, UPPER_RED_THRESHOLD)
    
    
    contours_red, _ = cv2.findContours(img_thresh_red[ROI_MIDDLE[1]:ROI_MIDDLE[3], ROI_MIDDLE[0]:ROI_MIDDLE[2]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours_green, _ = cv2.findContours(img_thresh_green[ROI_MIDDLE[1]:ROI_MIDDLE[3], ROI_MIDDLE[0]:ROI_MIDDLE[2]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


    b_mask = cv2.inRange(img_hsv, LOWER_BLUE_THRESHOLD, UPPER_BLUE_THRESHOLD)

    #find blue contours to detect the lines on the mat
    contours_blue,_ = cv2.findContours(b_mask[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    o_mask = cv2.bitwise_or(cv2.inRange(img_hsv, LOWER_ORANGE_THRESHOLD1, UPPER_ORANGE_THRESHOLD1), cv2.inRange(img_hsv, LOWER_ORANGE_THRESHOLD2, UPPER_ORANGE_THRESHOLD2))

    #find orange contours to detect the lines on the mat
    contours_orange,_ = cv2.findContours(o_mask[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    
    # define functions for right and left contours in respective ROIs
    left_contours_top, hierarchy = cv2.findContours(img_thresh[ROI_LEFT_TOP[1]:ROI_LEFT_TOP[3], ROI_LEFT_TOP[0]:ROI_LEFT_TOP[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    right_contours_top, hierarchy = cv2.findContours(img_thresh[ROI_RIGHT_TOP[1]:ROI_RIGHT_TOP[3], ROI_RIGHT_TOP[0]:ROI_RIGHT_TOP[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    left_contours_bot, hierarchy = cv2.findContours(img_thresh[ROI_LEFT_BOT[1]:ROI_LEFT_BOT[3], ROI_LEFT_BOT[0]:ROI_LEFT_BOT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    right_contours_bot, hierarchy = cv2.findContours(img_thresh[ROI_RIGHT_BOT[1]:ROI_RIGHT_BOT[3], ROI_RIGHT_BOT[0]:ROI_RIGHT_BOT[2]],
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    middle_contours, hierarchy = cv2.findContours(img_thresh[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]],
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
    middle_area = 0
    contours_coloured = [(contours_red,(0,0,255),1,ROI_MIDDLE),(contours_green,(0,255,0),1,ROI_MIDDLE),(contours,(0,0,0),1,(0,0)),(contours_orange,(52,140,235),0,ROI4),(contours_blue,(235,67,52),0,ROI4)]
    max_red_contour = 0
    max_green_contour = 0


    for c in contours_coloured:
        cont = c[0]
        colour = c[1]
        ROIROI = c[3]
        for i,cnt in enumerate(cont):
            area = cv2.contourArea(cnt)
            if area > 100:
                
                cnt[:, :, 0] += ROIROI[0]  # Add X offset
                cnt[:, :, 1] += ROIROI[1]  # Add Y offset

                cv2.drawContours(im, [cnt], -1, colour, 2)
            if colour == (0,0,0):
                if area > 200:
                    cv2.drawContours(im, contours, i, (0, 255, 255), 1)
            elif area > 1000:
                Board.RGB.setPixelColor(c[2], Board.PixelColor(colour[2],colour[1],colour[0]))
            else:
                Board.RGB.setPixelColor(c[2], Board.PixelColor(0,0,0))

            Board.RGB.show()
            
    max_green_contour,max_red_contour,direction = 0,0,""
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
        
    x,y,w,h = 0,0,0,0

    

    for i in contours_red:
        area = cv2.contourArea(i)
        if area > max_red_contour:
            
            max_red_contour = area
            if max_red_contour > PILLAR_SIZE and max_green_contour <= max_red_contour:
                approx=cv2.approxPolyDP(i, 0.01*cv2.arcLength(i,True),True)
                x,y,w,h=cv2.boundingRect(approx)
                direction = "red"
        
    for i in contours_green:
        area = cv2.contourArea(i)
        if area > max_green_contour:
            max_green_contour = area
            if max_green_contour > PILLAR_SIZE and max_green_contour > max_red_contour:
                approx=cv2.approxPolyDP(i, 0.01*cv2.arcLength(i,True),True)
                x,y,w,h=cv2.boundingRect(approx)
                direction = "green"
    if direction == "none":
        target = "none"
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

    for i in range(len(middle_contours)):
        cnt = middle_contours[i]

        area = cv2.contourArea(cnt)
        
        right_area_bot = max(area, middle_area)
        
    right_area = right_area_bot+right_area_top
    left_area = left_area_bot+left_area_top
    # draw all relatively large contours for debug utility
    """for i in range(len(contours)):
            cnt = contours[i]
            area = cv2.contourArea(cnt)
            if area > 100:
                cv2.drawContours(im, contours, i, (0, 255, 255), 1)"""
    


    if lastLapTurnAround:
        #point turn code
        Board.setPWMServoPulse(6, 1650, 100) 
        Board.setPWMServoPulse(1, pwm(MID_SERVO+MAX_TURN_DEGREE), 1000)
        time.sleep(0.5)
        Board.setPWMServoPulse(6, 1350, 100) 
        Board.setPWMServoPulse(1, pwm(MID_SERVO-MAX_TURN_DEGREE), 1000)
        time.sleep(0.5)
        Board.setPWMServoPulse(6, 1650, 100) 
        Board.setPWMServoPulse(1, pwm(MID_SERVO+MAX_TURN_DEGREE), 1000)
        time.sleep(0.5)
        Board.setPWMServoPulse(6, 1350, 100) 
        Board.setPWMServoPulse(1, pwm(MID_SERVO-MAX_TURN_DEGREE), 1000)
        time.sleep(0.5)
        lastLapTurnAround = False
        continue

    """
    if target != 'none':
        if target > x-10 and target < x+10:
            target = 'none'
    """
    if direction == "red":
        prevPillar = "red"
        target = RED_TARGET
        
    elif direction == "green":
        prevPillar = "green"
        target = GREEN_TARGET
    

    if trackDir == "none":
        if max_blue_area>max_orange_area and max_blue_area > LINE_THRESHOLD:
            trackDir = "left"
        if max_orange_area > max_blue_area and max_orange_area > LINE_THRESHOLD:
            trackDir = "right"

    
    elif trackDir == "right":
        if (turnDir == "right" and max_blue_area < LINE_THRESHOLD and max_orange_area < LINE_THRESHOLD and right_area > 200):          
            turnDir = "none"
            print("done turning")
                
        elif max_orange_area > LINE_THRESHOLD:
    
            #if the turn direction hasn't been changed yet change the turn direction to right
            if turnDir == "none":
                total_turn += 1
                turnDir = "right"    
                print(turnDir,total_turn)

        
            
    elif trackDir == "left":

        if (turnDir == "left" and max_orange_area < LINE_THRESHOLD and max_blue_area < LINE_THRESHOLD  and left_area > 200):
            
            turnDir = "none"
            print("done turning")
                
        elif max_blue_area > LINE_THRESHOLD:
                #if the turn direction hasn't been changed yet change the turn direction to left
            if turnDir == "none":
                total_turn+= 1
                turnDir = "left" 
                print(turnDir,total_turn)


    if target != 'none':
        last_difference = 0
        
        
            
        temp_dist = math.dist([x + w // 2, y], [320, 480])
              
              #print(temp_dist, "pixels away")
              
              #if the pillar is close enough add it to the number of pillars
        if max_green_contour > PILLAR_SIZE and temp_dist < 395:
            num_pillars_g += 1
        
        if max_red_contour > PILLAR_SIZE and temp_dist < 395:
            num_pillars_r += 1
        
        if ((max_green_contour > PILLAR_SIZE or max_red_contour > PILLAR_SIZE)):
            #print(area)
            if (max_green_contour > PILLAR_SIZE and max_red_contour > PILLAR_SIZE):
                OBSTACLEPG = 0.001
                YAXISPG = 0.02
                print("two pillars")
                
            elif (max_green_contour > PILLAR_SIZE):
                
                if h*w > 8500 and (x+(w/2)) < 460:
                    servo_angle = MID_SERVO
                    time.sleep(0.3)

                    pw = pwm(servo_angle)
                    
                    Board.setPWMServoPulse(6, 1500, 100) 
                    Board.setPWMServoPulse(1, pw, 1000) 
                    Board.setPWMServoPulse(6, 1590, 100) 
                    time.sleep(2.5)
                    
            elif (max_red_contour > PILLAR_SIZE):
                if h*w > 8500 and (x+(w/2)) > 280:
                    servo_angle = MID_SERVO
                    time.sleep(0.3)

                    pw = pwm(servo_angle)
                    Board.setPWMServoPulse(6, 1500, 100) 
                    Board.setPWMServoPulse(1, pw, 1000) 

                    Board.setPWMServoPulse(6, 1590, 100) 
                    time.sleep(2.5)
                  
            if turnDir != 'none':
                OBSTACLEPG = 0.0033
                
            dc_speed = DC_TURN_SPEED
            error = target - (x+(w/2))
            
            servo_angle = MID_SERVO + ((((error) * MAX_TURN_DEGREE) * OBSTACLEPG)) + (error - prevError) * OBSTACLEPD
            if error <= 0:
                servo_angle -= int(YAXISPG * (y+h - 120))  
            else:
                servo_angle += int(YAXISPG * (y+h - 120))
        
        else:
            target = 'none'
                
                
    else:
        
                    
        if turnDir == 'right':
            prevError = 0
            last_difference = 0

            servo_angle = MID_SERVO-MAX_TURN_DEGREE 
            dc_speed = DC_TURN_SPEED
            
                
        elif turnDir == 'left':
            prevError = 0
            last_difference = 0
            servo_angle = MID_SERVO+MAX_TURN_DEGREE 
            dc_speed = DC_TURN_SPEED
        
        
        
        elif middle_area > 80:

                #if the last pillar the car passed was green take a hard right turn and if the last pillar was red take a hard left turn
                if last_target == GREEN_TARGET:
                    angle = MID_SERVO-MAX_TURN_DEGREE 
                    #print("green")
                elif last_target == RED_TARGET:
                    angle = MID_SERVO+MAX_TURN_DEGREE
                    #print("red")
                    
        
        else:
            turnDir = "none"
            dc_speed = DC_STRAIGHT_SPEED
            error = 0
            
            

            # if in the straight section, calculate the current_difference between the contours in the left and right area
            current_difference = left_area - right_area
            #print ("current current_difference: " + str(current_difference))
           
            #calculate steering amount using preportional-derivative steering
            # multiply the current_difference by a constant variable and add the projected error multiplied by another constand
            if last_difference != 0 and left_area != 0 and right_area != 0:
                servo_angle = MID_SERVO - (current_difference * PG + (current_difference-last_difference) * PD)
            else:
                servo_angle = MID_SERVO - (current_difference * PG)
            #print (MID_SERVO - (current_difference * PG + (current_difference-last_difference) * PD))
            
           
            
    
    print("OBSTACLE PG: " + str((error) * MAX_TURN_DEGREE * OBSTACLEPG))
    print("OBSTACLE PD: " + str((error - prevError) * OBSTACLEPD))
    
    if error <= 0:
        print("Y AXIS MULTIPLIER: " + str((-1* YAXISPG * ((y+h - 120)))))
       
    else:
        print("Y AXIS MULTIPLIER: " + str(( YAXISPG * y+h - 120)))
    print("TOTAL ANGLE " + str(servo_angle))
    
    #   #if the total turns has surpassed the amount required, increment the action counter by 1
    if total_turn == MAX_TURNS:
        action_counter += 1
        
    # set the last current_difference equal to the current current_difference for derivative steering
    last_difference= current_difference
    prevError = error
    last_target = target
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
    
    image = cv2.line(im, (ROI_MIDDLE[0], ROI_MIDDLE[1]), (ROI_MIDDLE[2], ROI_MIDDLE[1]), (0, 255, 255), 2)
    image = cv2.line(im, (ROI_MIDDLE[0], ROI_MIDDLE[1]), (ROI_MIDDLE[0], ROI_MIDDLE[3]), (0, 255, 255), 2)
    image = cv2.line(im, (ROI_MIDDLE[2], ROI_MIDDLE[3]), (ROI_MIDDLE[2], ROI_MIDDLE[1]), (0, 255, 255), 2)
    image = cv2.line(im, (ROI_MIDDLE[2], ROI_MIDDLE[3]), (ROI_MIDDLE[0], ROI_MIDDLE[3]), (0, 255, 255), 2)

    image = cv2.line(im, (x, y), (x+w, y), (0, 255, 255), 1)
    image = cv2.line(im, (x, y), ((x, h+y)), (0, 255, 255), 1)
    image = cv2.line(im, (x+w, y), (x+w, y+h), (0, 255, 255), 1)
    image = cv2.line(im, (x, y+h), (x+w, y+h), (0, 255, 255), 1)

    image = cv2.line(im, (ROI4[0], ROI4[1]), (ROI4[2], ROI4[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI4[0], ROI4[1]), (ROI4[0], ROI4[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI4[2], ROI4[3]), (ROI4[2], ROI4[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI4[2], ROI4[3]), (ROI4[0], ROI4[3]), (0, 255, 255), 4)      
    if target != 'none':
        image = cv2.line(im, (target, 0), (target, 520), (255, 255, 0), 1)
    cv2.circle(im,(int(x+(w/2)),y),5,(255,255,0),1,-1)
    # display the camera
    cv2.imshow("Camera", im)
    
    
    # if the number of actions to the straight section has been met, stop the car
    if action_counter >= ACTIONS_TO_STRAIGHT:
        if not lastLapDone:
            if prevPillar == "green":
                lastLapTurnAround = True
                stop()
                time.sleep(1)
            action_counter = 0
            total_turn = 0
            MAX_TURNS = 4
        
        else:
            #parking lot thing sequence
            pass


    if (cv2.waitKey(1)==ord("q")):#
        time.sleep(0.02)
        stop()
        print("stop")
        
        break
    

cv2.destroyAllWindows()

