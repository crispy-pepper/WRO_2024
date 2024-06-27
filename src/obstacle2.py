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
MAX_TURN_DEGREE = 40
ROI_LEFT_BOT = [0, 290, 100, 330]
ROI_RIGHT_BOT = [540, 290, 640, 330]
ROI_LEFT_TOP = [0, 270, 50, 290]
ROI_RIGHT_TOP = [590, 270, 640, 290]
ROI4 = [200, 250, 440, 300]
ROI_MIDDLE = [0, 220, 640, 380]

PD = 0.0036
PG = 0.009
WIDTH = 640
HEIGHT = 480
POINTS = [(115,200), (525,200), (640,370), (0,370)]
LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 70])
LOWER_RED_THRESHOLD1 = np.array([0, 50, 50])
UPPER_RED_THRESHOLD1 = np.array([10, 255, 255])
LOWER_RED_THRESHOLD2 = np.array([167, 50, 50])
UPPER_RED_THRESHOLD2 = np.array([180, 255, 255])
#LOWER_GREEN_THRESHOLD = np.array([58, 42, 60])
#UPPER_GREEN_THRESHOLD = np.array([116, 255, 255])
LOWER_GREEN_THRESHOLD = np.array([58, 62, 55])
UPPER_GREEN_THRESHOLD = np.array([96, 255, 255])
LOWER_ORANGE_THRESHOLD = np.array([0, 100, 175])
UPPER_ORANGE_THRESHOLD = np.array([25, 255, 255])
LOWER_BLUE_THRESHOLD = np.array([100, 100, 100])
UPPER_BLUE_THRESHOLD = np.array([135, 255, 255])


PILLAR_SIZE = 3200







DC_STRAIGHT_SPEED = 1350
DC_TURN_SPEED = 1364
MAX_TURNS = 12
ACTIONS_TO_STRAIGHT = 400
WALL_THRESHOLD = 600
NO_WALL_THRESHOLD = 50

#dynamic variables
sharp_turn_left = False
sharp_turn_right = False
total_turn = 0
action_counter = 0
last_difference = 0
current_difference = 0
servo_angle=0 
lastLapTurnAround = False
prevPillar = ""

turning_iter = 0
turnDir = ""
lastLapDone = False

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

def stop():
    Board.setPWMServoPulse(6, 1500, 100) 
    Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)

    

Board.setPWMServoPulse(1, pwm(MID_SERVO), 10) #turn servo to mid
Board.setPWMServoPulse(6, 1500, 100) # arm the esc motor
time.sleep(2)
print("---------------------------- running--------------------------")

while True:
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
    
    
    contours_red, _ = cv2.findContours(img_thresh_red[ROI_MIDDLE[1]:ROI_MIDDLE[3], ROI_MIDDLE[0]:ROI_MIDDLE[2]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours_green, _ = cv2.findContours(img_thresh_green[ROI_MIDDLE[1]:ROI_MIDDLE[3], ROI_MIDDLE[0]:ROI_MIDDLE[2]], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)


    b_mask = cv2.inRange(img_hsv, LOWER_BLUE_THRESHOLD, UPPER_BLUE_THRESHOLD)
    #find blue contours to detect the lines on the mat
    contours_blue = cv2.findContours(b_mask[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    o_mask = cv2.inRange(img_hsv, LOWER_ORANGE_THRESHOLD, UPPER_ORANGE_THRESHOLD)

    #find orange contours to detect the lines on the mat
    contours_orange = cv2.findContours(o_mask[ROI4[1]:ROI4[3], ROI4[0]:ROI4[2]], cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)[-2]
    
    
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
    contours_coloured = [(contours_red,(0,0,255),1),(contours_green,(0,255,0),1),(contours,(0,0,0),1)]
    max_red_contour = 0
    max_green_contour = 0


    for c in contours_coloured:
        cont = c[0]
        colour = c[1]
        for i,cnt in enumerate(cont):
            area = cv2.contourArea(cnt)
            if area > 100:
                
                cnt[:, :, 0] += ROI_MIDDLE[0]  # Add X offset
                cnt[:, :, 1] += ROI_MIDDLE[1]  # Add Y offset

                cv2.drawContours(im, [cnt], -1, colour, 2)
            if colour == (0,0,0):
                if area > 200:
                    cv2.drawContours(im, contours, i, (0, 255, 255), 1)
            elif area > 2000:
                Board.RGB.setPixelColor(c[2], Board.PixelColor(colour[2],colour[1],colour[0]))
            else:
                Board.RGB.setPixelColor(c[2], Board.PixelColor(0,0,0))

            Board.RGB.show()
            
    max_green_contour,max_red_contour,direction = 0,0,""
    x,y,w,h = 0,0,0,0

    for i in contours_red:
        area = cv2.contourArea(i)
        if area > max_red_contour:
            approx=cv2.approxPolyDP(i, 0.01*cv2.arcLength(i,True),True)
            x,y,w,h=cv2.boundingRect(approx)
            max_red_contour = area
            if max_red_contour > 800:
                direction = "red"
        
    for i in contours_green:
        area = cv2.contourArea(i)
        if area > max_green_contour:
            max_green_contour = area
            if max_green_contour > max_red_contour:
                approx=cv2.approxPolyDP(i, 0.01*cv2.arcLength(i,True),True)
                x,y,w,h=cv2.boundingRect(approx)
                direction = "green"
        
    
    # loop to find largest contours
    
    """for i in range(len(left_contours_top)):
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
        
        right_area_bot = max(area, right_area_bot)"""
    left_area_bot = max(map(cv2.contourArea, left_contours_bot))
    left_area_top = max(map(cv2.contourArea, left_contours_top))
    right_area_bot = max(map(cv2.contourArea, right_contours_bot))
    right_area_top = max(map(cv2.contourArea, right_contours_top))
        
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

    # set default DC motor speed as straight section speed
    target = 0
    dc_speed = DC_STRAIGHT_SPEED
    if direction == "red":
        prevPillar = "red"
        target = 130
    elif direction == "green":
        prevPillar = "green"
        target = 480
    
    ''' if (direction == "red" and max_red_contour > 1200):
        servo_angle = MID_SERVO - MAX_TURN_DEGREE
        dc_speed = DC_TURN_SPEED
        print("Detected RED, turning RIGHT")
        """if max_red_contour > PILLAR_SIZE and x <= 200:
            servo_angle = MID_SERVO + MAX_TURN_DEGREE
            print("Centering")"""

    elif direction == "green" and max_green_contour > 1200:
        servo_angle = MID_SERVO + MAX_TURN_DEGREE
        dc_speed = DC_TURN_SPEED
        print("Detected GREEN, turning LEFT")
        """if (max_green_contour > PILLAR_SIZE and x >= 440):

            servo_angle = MID_SERVO - MAX_TURN_DEGREE
            print("Centering")""" '''
    if (max_green_contour > 800 or max_red_contour > 800):
        if x < target:
            servo_angle = MID_SERVO + MAX_TURN_DEGREE
        elif x > target:
            servo_angle = MID_SERVO - MAX_TURN_DEGREE
        """ else:
            if servo_angle > MID_SERVO:
                servo_angle = MID_SERVO - MAX_TURN_DEGREE
            else:cd 
                servo_angle = MID_SERVO + MAX_TURN_DEGREE"""
    
    else:
        if ((sharp_turn_right and right_area< WALL_THRESHOLD) or 1 <= turning_iter<=200):
            servo_angle = MID_SERVO-MAX_TURN_DEGREE
            #dc_speed = DC_TURN_SPEED
            turning_iter += 1
                
        elif ((sharp_turn_left and left_area< WALL_THRESHOLD)or 1 <= turning_iter <= 200):
            servo_angle = MID_SERVO+MAX_TURN_DEGREE
            #dc_speed = DC_TURN_SPEED
            turning_iter += 1
        
        
        else:
            sharp_turn_left = False
            sharp_turn_right = False
            turning_iter = 0
            if right_area < NO_WALL_THRESHOLD:
                #print("no wall to the right") 
                
                # set all movement variables to turn sharply right
                total_turn+=1
                
                print(str(total_turn) + "th turn") 
                turning_iter += 1
                sharp_turn_right = True
                

                
            elif left_area < NO_WALL_THRESHOLD:
                #print("no wall to the left")
                
                # set all movement variables to turn sharply left
                sharp_turn_left = True
                total_turn+=1
                
                print(str(total_turn) + "th turn") 
                turning_iter += 1


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
    image = cv2.line(im, (ROI_LEFT_BOT[2], ROI_LEFT_BOT[3]), (ROI_LEFT_BOT[2
           ], ROI_LEFT_BOT[1]), (0, 255, 255), 4)
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

    image = cv2.line(im, (target, 0), (target, 520), (255, 255, 0), 1)
    cv2.circle(im,(x,y),5,(255,255,0),1,-1)
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
