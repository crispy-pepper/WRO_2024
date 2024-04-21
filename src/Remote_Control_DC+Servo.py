
import sys
sys.path.append('/home/pi/TurboPi/')
import time
import HiwonderSDK.Board as Board
from readchar import readkey, key

Board.setPWMServoPulse(1, 1500, 10) #Turn to 90 degree
# 'Arm' the ESC
Board.setPWMServoPulse(6, 1500, 100) 
time.sleep(6)
print("Ready\n")


if __name__ == '__main__':
    b = 1500
    s = 90
    while True:
        k = readkey()
        #remote control system
         
        if k == ' ':
            b = 1500
            s = 90
            print("stop")
        elif k == 'a':
            s = 0
            print("left")
        elif k == 'd':
            s = 180
            print("right")
        elif k == 'w':
            b = 1550
            print("forward")
        elif k == 's':
            b = 1400
            print("backward")
        elif k == 'x':
            s = 90
            print("straight")
        elif k == 'q':
            if b >= 1500:
                b += 50
                if b>2000:
                    b = 2000
            else:
                b -= 50
                if b <1000:
                    b = 1000
            print(f"speed up: {b}")
        elif k == 'e':
            if b >= 1500:
                b -= 50
                if b < 1500:
                    b = 1500
            else:
                b += 50
                if b > 1500:
                    b = 1500
            print(f"speed down: {b}")
        #bldc
        pw = int(11.1*s+500)
        Board.setPWMServoPulse(6, b, 100) 
        Board.setPWMServoPulse(1, pw, 1000)
        time.sleep(0.01)
