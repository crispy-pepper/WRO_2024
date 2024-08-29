
#libraries
import sys
import cv2
import numpy as np
import time
sys.path.append('/home/pi/TurboPi/')
from picamera2 import Picamera2
import HiwonderSDK.Board as Board


MID_SERVO = 80
MAX_TURN_DEGREE = 40
# utility function for pwm angle calculation
def pwm(degree):
	return round(degree*11.1 + 500)
	
def pause(times):
    Board.setPWMServoPulse(6, 1500, 100) 
    Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
    time.sleep(times)

bspeed = 1650
fspeed = 1250
    

Board.setPWMServoPulse(1, pwm(MID_SERVO), 10) #turn servo to mid
Board.setPWMServoPulse(6, 1500, 100) # arm the esc motor
time.sleep(2)
print("---------------------------- running--------------------------")


for i in range(2):

    Board.setPWMServoPulse(6, bspeed, 100) 
    Board.setPWMServoPulse(1, pwm(MID_SERVO+MAX_TURN_DEGREE), 1000)

    time.sleep(1)

    pause(1)

    time.sleep(0.1)
    Board.setPWMServoPulse(6, fspeed, 100) 
    Board.setPWMServoPulse(1, pwm(MID_SERVO-MAX_TURN_DEGREE), 1000)

    time.sleep(1)

    pause(1)



time.sleep(0.5)

Board.setPWMServoPulse(6, 1500, 100) 
Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
time.sleep(1)
