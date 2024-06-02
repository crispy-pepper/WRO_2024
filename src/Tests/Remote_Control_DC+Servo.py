
import sys
sys.path.append('/home/pi/TurboPi/')
import time
import HiwonderSDK.Board as Board
from readchar import readkey, key


def pwm(degree):
	return round(degree*11.1 + 500)
mid = 80
max_turn_degree = 32
Board.setPWMServoPulse(1, pwm(mid), 10) #Turn to 90 degree
# 'Arm' the ESC
Board.setPWMServoPulse(6, 1500, 100) 
time.sleep(6)
print("Ready\n")


if __name__ == '__main__':
    b = 1500
    s = mid
    while True:
        k = readkey()
        #remote control system
         
        if k == ' ':
            Board.setPWMServoPulse(5, 1500, 100) 
            Board.setPWMServoPulse(1, pwm(mid), 1000)
            time.sleep(0.01)
            Board.setPWMServoPulse(1, 0, 1000)
            time.sleep(0.01)
            print("stop")
            break
        elif k == 'a':
            s = mid+max_turn_degree
            print("left")
        elif k == 'd':
            s = mid-max_turn_degree
            print("right")
        elif k == 'w':
            b = 1550
            print("forward")
        elif k == 's':
            b = 1400
            print("backward")
        elif k == 'x':
            s = mid
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
        pw = pwm(s)
        Board.setPWMServoPulse(6, b, 100) 
        Board.setPWMServoPulse(1, pw, 1000)
        time.sleep(0.01)
