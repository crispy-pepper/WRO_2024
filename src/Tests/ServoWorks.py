import sys
sys.path.append('/home/pi/TurboPi/')
import time
import HiwonderSDK.Board as Board

Board.setPWMServoPulse(1, 1500, 1) #Turn to 90 degree
time.sleep(1)

angle = 90
pw = 11.1 * angle + 500

if __name__ == '__main__':
    while(angle >= 60): #Turn from 90 to 60 gradually
        angle -= 1
        pw = 11.1 * angle + 500
        Board.setPWMServoPulse(1, int(pw), 1)
        time.sleep(0.01)

    while(angle <= 120): #Turn from 60 to 120 gradually
        angle += 1
        pw = 11.1 * angle + 500
        Board.setPWMServoPulse(1, int(pw), 1)
        time.sleep(0.01)

    while(angle >= 90): #Turn from 120 to 90 gradually
        angle -= 1
        pw = 11.1 * angle + 500
        Board.setPWMServoPulse(1, int(pw), 1)
        time.sleep(0.01)
        
    Board.setPWMServoPulse(1, 1500, 1)
    time.sleep(0.01)
    
    Board.setPWMServoPulse(1, 0, 1)