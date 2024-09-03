# libraries
import math
import sys
import cv2
import numpy as np
import time

sys.path.append("/home/pi/TurboPi/")
from picamera2 import Picamera2
import HiwonderSDK.Board as Board
import libcamera
from libcamera import controls


ROI_LEFT_BOT = [0, 290, 100, 345]
ROI_RIGHT_BOT = [540, 290, 640, 345]
ROI_LEFT_TOP = [0, 270, 50, 290]
ROI_RIGHT_TOP = [590, 270, 640, 290]
ROI4 = [285, 330, 355, 355]
ROI_MIDDLE = [0, 200, 640, 420]
ROI_PARKING_LEFT = [0, 225, 320, 345]
ROI_PARKING_RIGHT = [320, 225, 640, 345]


# proportion constants for the servo motor angle (PD steering) for the wall following algorithm

PD = 0.06
PG = 0.0058

# camera constants

WIDTH = 640
HEIGHT = 480
POINTS = [(115, 200), (525, 200), (640, 370), (0, 370)]

# colour thresholds

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 50])

LOWER_RED_THRESHOLD1 = np.array([5, 154, 80])
UPPER_RED_THRESHOLD1 = np.array([12, 255, 255])
LOWER_RED_THRESHOLD2 = np.array([180, 144, 107])
UPPER_RED_THRESHOLD2 = np.array([180, 255, 255])

LOWER_GREEN_THRESHOLD = np.array([45, 95, 50])
UPPER_GREEN_THRESHOLD = np.array([80, 255, 195])

LOWER_ORANGE_THRESHOLD1 = np.array([180, 100, 100])
UPPER_ORANGE_THRESHOLD1 = np.array([180, 255, 255])
LOWER_ORANGE_THRESHOLD2 = np.array([15, 175, 110])
UPPER_ORANGE_THRESHOLD2 = np.array([26, 255, 240])

LOWER_BLUE_THRESHOLD = np.array([34, 0, 49])
UPPER_BLUE_THRESHOLD = np.array([110, 100, 135])

LOWER_MAGENTA_THRESHOLD1 = np.array([0, 100, 50])
UPPER_MAGENTA_THRESHOLD1 = np.array([5, 215, 255])

LOWER_MAGENTA_THRESHOLD2 = np.array([166, 110, 50])
UPPER_MAGENTA_THRESHOLD2 = np.array([180, 255, 255])

LINE_THRESHOLD = 50  # minimum contour size of orange or blue line


# pillar constants


PILLAR_SIZE = 300


# motor constants

DC_SPEED = 1347
MID_SERVO = 80
MAX_TURN_DEGREE = 39


ACTIONS_TO_STRAIGHT = (
    100  # number of iterations to enter the straight section from the turning section
)

# pillar avoidance algorithm variables

prevPillar = ""
target = None
last_target = None
error = 0
prevError = 0

# variables used for last lap

noRed = False
need_space = False
tpt2red = False
lastLapTurnAround = False
lastLapContinue = False

# variables for turning algorithm

total_turn = 0
turnDir = None
trackDir = None
line_seen = False
turning_iter = 0
threeLaps = False  # used to determine when the robot has completed 3 laps and can stop
red_target = 110
green_target = 530

last_difference = 0  # used for derivative calculation for wall following
current_difference = 0  # used for derivative calculation for wall following
action_counter = 0  # counter to determine when the robot has entered the straight section from the turning section
servo_angle = 0


# variables used for parking
need_pause = True
parkingR = False
parkingL = False
parking_algorithm = False


# camera setup
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.set_controls(
    {
        "Brightness": 0.05,
        "AwbMode": libcamera.controls.AwbModeEnum.Custom,
        "ColourGains": (1.08, 1),
    }
)
picam2.start()


def drawROI(ROI):  # draw the rectangular ROI on the image
    """
    Displays/draws a rectanngular region of interest (ROI) on the OpenCV window

    Parameters:
    ROI (list): A list of 2 coodinates detailing the top left and the bottom right of the ROI

    Returns:

    """

    image = cv2.line(im, (ROI[0], ROI[1]), (ROI[2], ROI[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI[0], ROI[1]), (ROI[0], ROI[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI[2], ROI[3]), (ROI[2], ROI[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI[2], ROI[3]), (ROI[0], ROI[3]), (0, 255, 255), 4)


# utility function for pwm angle calculation
def pwm(degree):
    return round(degree * 11.1 + 500)


def stop():

    Board.setPWMServoPulse(6, 1500, 100)
    Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
    Board.setPWMServoPulse(6, 1500, 100)
    cv2.destroyAllWindows()
    sys.exit()
    return 0


Board.setPWMServoPulse(1, pwm(MID_SERVO), 1)  # turn servo to mid
Board.setPWMServoPulse(6, 1500, 100)  # arm the esc motor
time.sleep(1)
print("---------------------------- running--------------------------")

while True:

    # obstacle avoidance variables that are reset every iteration of the main loop

    # ROIs

    OBSTACLEPG = 0.0017  # proportional gain for obstacle avoidance

    OBSTACLEPD = 0.005  # derivative gain for obstacle avoidance

    YAXISPG = 0.017  # y-axis steering that is proportional to the y-axis of the pillar

    num_pillars_g = 0  # tracks how many green pillars are currently detected

    num_pillars_r = 0  # tracks how many red pillars are currently detected

    x, y, w, h = 0, 0, 0, 0  # variables for the bounding rectangle of the pillar

    closest_pillar_dist = 100000  # tracks the distance to the closest pillar (set to a large number if no pillar is detected)

    closest_pillar_x = None  # tracks the x-coordinate of the closest pillar

    closest_pillar_y = None  # tracks the y-coordinate of the closest pillar

    closest_pillar_area = None  # tracks the area of the closest pillar

    closest_pillar_colour = None  # tracks the colour of the closest pillar

    if parking_algorithm:
        PILLAR_SIZE = 100
        OBSTACLEPG = 0.0025

    # setup camera frame
    im = picam2.capture_array()
    input = np.float32(POINTS)
    output = np.float32(
        [(0, 0), (WIDTH - 1, 0), (WIDTH - 1, HEIGHT - 1), (0, HEIGHT - 1)]
    )

    # convert to hsv
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    # --------------------------------------------------- Set Image Thresholds ---------------------------------------------------
    img_thresh_black = cv2.inRange(
        img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD
    )

    img_thresh_green = cv2.inRange(
        img_hsv, LOWER_GREEN_THRESHOLD, UPPER_GREEN_THRESHOLD
    )
    img_thresh_red = cv2.bitwise_or(
        cv2.inRange(img_hsv, LOWER_RED_THRESHOLD1, UPPER_RED_THRESHOLD1),
        cv2.inRange(img_hsv, LOWER_RED_THRESHOLD2, UPPER_RED_THRESHOLD2),
    )
    img_thresh_blue = cv2.inRange(img_hsv, LOWER_BLUE_THRESHOLD, UPPER_BLUE_THRESHOLD)
    img_thresh_orange = cv2.bitwise_or(
        cv2.inRange(img_hsv, LOWER_ORANGE_THRESHOLD1, UPPER_ORANGE_THRESHOLD1),
        cv2.inRange(img_hsv, LOWER_ORANGE_THRESHOLD2, UPPER_ORANGE_THRESHOLD2),
    )

    img_thresh_magenta = cv2.bitwise_or(
        cv2.inRange(img_hsv, LOWER_MAGENTA_THRESHOLD1, UPPER_MAGENTA_THRESHOLD1),
        cv2.inRange(img_hsv, LOWER_MAGENTA_THRESHOLD2, UPPER_MAGENTA_THRESHOLD2),
    )

    # --------------------------------------------------- Find Contours ---------------------------------------------------

    contours_blue, _ = cv2.findContours(
        img_thresh_blue[ROI4[1] : ROI4[3], ROI4[0] : ROI4[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    contours_orange, _ = cv2.findContours(
        img_thresh_orange[ROI4[1] : ROI4[3], ROI4[0] : ROI4[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    left_contours_magenta, _ = cv2.findContours(
        img_thresh_magenta[
            ROI_PARKING_LEFT[1] : ROI_PARKING_LEFT[3],
            ROI_PARKING_LEFT[0] : ROI_PARKING_LEFT[2],
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    right_contours_magenta, _ = cv2.findContours(
        img_thresh_magenta[
            ROI_PARKING_RIGHT[1] : ROI_PARKING_RIGHT[3],
            ROI_PARKING_RIGHT[0] : ROI_PARKING_RIGHT[2],
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    center_contours_magenta, _ = cv2.findContours(
        img_thresh_magenta[
            ROI4[1] : ROI4[3],
            ROI4[0] : ROI4[2],
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    middle_contours_magenta, _ = cv2.findContours(
        img_thresh_magenta[
            ROI_MIDDLE[1] : ROI_MIDDLE[3], ROI_MIDDLE[0] : ROI_MIDDLE[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    maxArea = 0

    for i in middle_contours_magenta:
        area = cv2.contourArea(i)

        if area > maxArea and area > PILLAR_SIZE:
            maxArea = area

            approx = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            x += ROI_MIDDLE[0]
            y += ROI_MIDDLE[1]
            image = cv2.line(im, (x, y), (x + w, y), (255, 255, 255), 1)
            image = cv2.line(im, (x, y), ((x, h + y)), (255, 255, 255), 1)
            image = cv2.line(im, (x + w, y), (x + w, y + h), (255, 255, 255), 1)
            image = cv2.line(im, (x, y + h), (x + w, y + h), (255, 255, 255), 1)
            cv2.circle(im, (int(x + (w / 2)), y), 5, (255, 255, 0), 1, -1)

    for i in range((y - 10) if y >= 10 else (0), (y + h + 10) if y + h <= 470 else 480):
        for j in range(
            (x - 50) if x >= 50 else (0), (x + w + 50) if x + w <= 590 else 640
        ):
            img_thresh_red[i][j] = 0

    contours_red, _ = cv2.findContours(
        img_thresh_red[ROI_MIDDLE[1] : ROI_MIDDLE[3], ROI_MIDDLE[0] : ROI_MIDDLE[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    contours_green, _ = cv2.findContours(
        img_thresh_green[ROI_MIDDLE[1] : ROI_MIDDLE[3], ROI_MIDDLE[0] : ROI_MIDDLE[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    left_top_contours_black, hierarchy = cv2.findContours(
        img_thresh_black[
            ROI_LEFT_TOP[1] : ROI_LEFT_TOP[3], ROI_LEFT_TOP[0] : ROI_LEFT_TOP[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    right_top_contours_black, hierarchy = cv2.findContours(
        img_thresh_black[
            ROI_RIGHT_TOP[1] : ROI_RIGHT_TOP[3], ROI_RIGHT_TOP[0] : ROI_RIGHT_TOP[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    left_bot_contours_black, hierarchy = cv2.findContours(
        img_thresh_black[
            ROI_LEFT_BOT[1] : ROI_LEFT_BOT[3], ROI_LEFT_BOT[0] : ROI_LEFT_BOT[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    right_bot_contours_black, hierarchy = cv2.findContours(
        img_thresh_black[
            ROI_RIGHT_BOT[1] : ROI_RIGHT_BOT[3], ROI_RIGHT_BOT[0] : ROI_RIGHT_BOT[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    middle_contours_black, hierarchy = cv2.findContours(
        img_thresh_black[ROI4[1] : ROI4[3], ROI4[0] : ROI4[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    contours, hierarchy = cv2.findContours(
        img_thresh_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )

    # --------------------------------------------------- Find Contour Areas ---------------------------------------------------

    left_area_top = 0
    left_area_bot = 0
    right_area_top = 0
    right_area_bot = 0
    middle_area = 0

    max_green_contour, max_red_contour = 0, 0
    max_blue_area = 0
    max_orange_area = 0
    max_red_contour = 0
    max_green_contour = 0

    contours_coloured = [
        (contours_red, (0, 0, 255), 1, ROI_MIDDLE),
        (contours_green, (0, 255, 0), 1, ROI_MIDDLE),
        (contours, (0, 255, 255), 1, (0, 0)),
        (contours_orange, (52, 140, 235), 0, ROI4),
        (contours_blue, (235, 67, 52), 0, ROI4),
        (left_contours_magenta, (255, 0, 255), 0, ROI_PARKING_LEFT),
        (right_contours_magenta, (255, 0, 255), 0, ROI_PARKING_RIGHT),
    ]
    for i in range(len(left_top_contours_black)):
        cnt = left_top_contours_black[i]
        area = cv2.contourArea(cnt)
        left_area_top = max(area, left_area_top)

    for i in range(len(left_bot_contours_black)):
        cnt = left_bot_contours_black[i]
        area = cv2.contourArea(cnt)
        left_area_bot = max(area, left_area_bot)

    for i in range(len(right_top_contours_black)):
        cnt = right_top_contours_black[i]

        area = cv2.contourArea(cnt)

        right_area_top = max(area, right_area_top)

    for i in range(len(right_bot_contours_black)):
        cnt = right_bot_contours_black[i]

        area = cv2.contourArea(cnt)

        right_area_bot = max(area, right_area_bot)

    for i in range(len(middle_contours_black)):
        cnt = middle_contours_black[i]

        area = cv2.contourArea(cnt)

        middle_area = max(area, middle_area)

    right_area = right_area_bot + right_area_top
    left_area = left_area_bot + left_area_top

    for c in contours_coloured:
        cont = c[0]
        colour = c[1]
        ROIROI = c[3]
        for i, cnt in enumerate(cont):
            area = cv2.contourArea(cnt)

            if area > 100:

                cnt[:, :, 0] += ROIROI[0]
                cnt[:, :, 1] += ROIROI[1]

                cv2.drawContours(im, [cnt], -1, colour, 2)

            Board.RGB.show()

    for i in range(len(contours_orange)):
        cnt = contours_orange[i]
        max_orange_area = max(cv2.contourArea(cnt), max_orange_area)
        cnt[:, :, 0] += ROI4[0]
        cnt[:, :, 1] += ROI4[1]
        cv2.drawContours(im, contours_orange, i, (255, 255, 0), 1)

    for i in range(len(contours_blue)):
        cnt = contours_blue[i]
        max_blue_area = max(cv2.contourArea(cnt), max_blue_area)
        cnt[:, :, 0] += ROI4[0]
        cnt[:, :, 1] += ROI4[1]

        cv2.drawContours(im, contours_blue, i, (255, 255, 0), 1)
    magenta_area_center = 0
    for i in range(len(center_contours_magenta)):
        cnt = center_contours_magenta[i]
        magenta_area_center = max(cv2.contourArea(cnt), magenta_area_center)
        cnt[:, :, 0] += ROI4[0]
        cnt[:, :, 1] += ROI4[1]

    """
     for i in range(len(left_contours_magenta)):
        cnt = left_contours_magenta[i]
        magenta_area_left = max(cv2.contourArea(cnt), magenta_area_left)

        cnt[:, :, 0] += ROI_PARKING_LEFT[0]
        cnt[:, :, 1] += ROI_PARKING_LEFT[1]
        cv2.drawContours(im, left_contours_magenta, i, (255, 0, 255), 1)

    for i in range(len(right_contours_magenta)):
        cnt = right_contours_magenta
        magenta_area_right = max(cv2.contourArea(cnt), magenta_area_right)
        cnt[:, :, 0] += ROI_PARKING_RIGHT[0]
        cnt[:, :, 1] += ROI_PARKING_RIGHT[1]
        cv2.drawContours(im, right_contours_magenta, i, (255, 0, 255), 1)

    
    
    
    
   
        cv2.drawContours(im, center_contours_magenta, i, (255, 0, 255), 1)
    """

    """
    if not parking_algorithm:
        right_area += magenta_area_right
        left_area += magenta_area_left
    
    """

    # --------------------------------------------------- Red Pillar Detection ---------------------------------------------------

    for i in contours_red:
        area = cv2.contourArea(i)

        if area > PILLAR_SIZE:

            approx = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            temp_dist = math.dist([x + w // 2, y], [320, 480])
            # temp_dist = 480 - y

            if temp_dist < 500:
                image = cv2.line(im, (x, y), (x + w, y), (0, 255, 255), 1)
                image = cv2.line(im, (x, y), ((x, h + y)), (0, 255, 255), 1)
                image = cv2.line(im, (x + w, y), (x + w, y + h), (0, 255, 255), 1)
                image = cv2.line(im, (x, y + h), (x + w, y + h), (0, 255, 255), 1)
                cv2.circle(im, (int(x + (w / 2)), y), 5, (255, 255, 0), 1, -1)

                num_pillars_r += 1

                if temp_dist < closest_pillar_dist:
                    if y + h > 440 and x + (w // 2) > 240:
                        # pillar has already been passed
                        pass
                    elif y + h < 200:

                        pass
                    else:
                        # set closest pillar variables
                        closest_pillar_dist = temp_dist
                        closest_pillar_colour = "red"
                        closest_pillar_x = x + w // 2
                        closest_pillar_y = y
                        closest_pillar_area = h * w

    # --------------------------------------------------- Green Pillar Detection -----------------------------------------------

    for i in contours_green:
        area = cv2.contourArea(i)

        if area > PILLAR_SIZE:

            approx = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, True), True)
            x, y, w, h = cv2.boundingRect(approx)
            temp_dist = math.dist([x + w // 2, y], [320, 480])
            # temp_dist = 480 - y
            if temp_dist < 500:
                image = cv2.line(im, (x, y), (x + w, y), (0, 255, 255), 1)
                image = cv2.line(im, (x, y), ((x, h + y)), (0, 255, 255), 1)
                image = cv2.line(im, (x + w, y), (x + w, y + h), (0, 255, 255), 1)
                image = cv2.line(im, (x, y + h), (x + w, y + h), (0, 255, 255), 1)
                cv2.circle(im, (int(x + (w / 2)), y), 5, (255, 255, 0), 1, -1)
                num_pillars_g += 1

                if temp_dist < closest_pillar_dist:
                    if y + h > 440 and x + (w // 2) > 400:
                        # pillar is too close
                        pass
                    elif y + h < 200:

                        pass
                    else:
                        # set closest pillar variables
                        closest_pillar_dist = temp_dist
                        closest_pillar_colour = "green"
                        closest_pillar_x = x + w // 2
                        closest_pillar_y = y
                        closest_pillar_area = h * w
    # --------------------------------------------------- Set Target ---------------------------------------------------

    if closest_pillar_colour == None:
        target = None
    elif closest_pillar_colour == "red":
        prevPillar = "red"
        target = red_target
    elif closest_pillar_colour == "green":
        prevPillar = "green"
        target = green_target

    # --------------------------------------------------- Parking Algorithm ---------------------------------------------------

    if parking_algorithm == True:
        info = [[0, 0, ROI_PARKING_LEFT], [0, 0, ROI_PARKING_RIGHT], [0, 0, ROI4]]
        conts = [left_contours_magenta, right_contours_magenta, center_contours_magenta]

        # finds the largest magenta contour in the left and right regions of interest and the y-coordinates
        for i in range(len(conts)):
            for x in range(len(conts[i])):
                cnt = conts[i][x]
                area = cv2.contourArea(cnt)

                if area > 200:

                    # get width, height, and x and y coordinates by bounding rect
                    approx = cv2.approxPolyDP(
                        cnt, 0.01 * cv2.arcLength(cnt, True), True
                    )
                    x, y, w, h = cv2.boundingRect(approx)

                    # since the x and y coordinates are the coordinates just in the ROI, add to the x and y values to make it the proper coordinates on the overall image
                    x += info[i][2][0]
                    y += info[i][2][1] + h

                    # replace largest contour
                    if area > info[i][0]:
                        info[i][0] = area
                        info[i][1] = y

            maxAreaL = info[0][0]  # biggest magenta contour on left ROI
            leftY = info[0][1]
            maxAreaR = info[1][0]  # biggest magenta contour on right ROI
            rightY = info[1][1]
            centerY = info[2][1]
            # conditions for initiating parking on the left side

            """
            if leftY >= 220 and maxAreaL > 100 and total_turn >= 12:
                if not parkingL and not parkingR:
                    Board.setPWMServoPulse(6, 1340, 100)
                    parkingL = True

                ROI4 = [250, 250, 390, 300]
            
            
            """

            if leftY >= 540 and maxAreaL > 150:
                if not parkingL and not parkingR:
                    Board.setPWMServoPulse(6, 1340, 100)
                    parkingL = True

                ROI4 = [250, 250, 390, 300]

            # conditions for initiating parking on the right side
            """
            if rightY >= 240 and maxAreaR > 100 and total_turn >= 12:
                if not parkingL and not parkingR:

                    parkingR = True
                    Board.setPWMServoPulse(6, 1340, 100)

                ROI4 = [250, 250, 390, 300]
            
            """
            # print(str(centerY) + " center y")
            # print(rightY)
            if rightY >= 534 and maxAreaR > 150:
                if not parkingL and not parkingR:

                    parkingR = True
                    Board.setPWMServoPulse(6, 1346, 100)

                ROI4 = [270, 390, 370, 420]

            if parkingR:
                print("a")
                # readjust if the parking lot is in front
                if centerY > 380:
                    print("trying to go back")
                    Board.setPWMServoPulse(6, 1500, 100)

                    Board.setPWMServoPulse(1, pwm(MID_SERVO + MAX_TURN_DEGREE), 1000)
                    time.sleep(0.1)

                    Board.setPWMServoPulse(6, 1580, 100)

                    time.sleep(0.4)
                    Board.setPWMServoPulse(6, 1500, 100)

                # turn right into parking lot
                else:
                    # if debug: LED1(255, 0, 255)

                    Board.setPWMServoPulse(1, pwm(MID_SERVO - MAX_TURN_DEGREE), 1000)

                    Board.setPWMServoPulse(6, 1350, 100)

            elif parkingL:

                if centerY > 315:

                    Board.setPWMServoPulse(6, 1500, 100)
                    time.sleep(0.1)
                    Board.setPWMServoPulse(6, 1580, 100)
                    Board.setPWMServoPulse(1, pwm(MID_SERVO - MAX_TURN_DEGREE), 1000)
                    time.sleep(0.5)
                    Board.setPWMServoPulse(6, 1500, 100)

                # turn right into parking lot
                else:
                    # if debug: LED1(255, 0, 255)

                    Board.setPWMServoPulse(6, 1350, 100)
                    Board.setPWMServoPulse(1, pwm(MID_SERVO + MAX_TURN_DEGREE), 1000)

            # if the area of the wall in front is above a limit stop as we are very close to the wall
            if middle_area > 1500:
                print("stop")
                Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
                Board.setPWMServoPulse(6, 1350, 100)
                time.sleep(1)
                stop()
                break

    # --------------------------------------------------- Three Point Turn ---------------------------------------------------

    if lastLapTurnAround:
        print("waiting for red to not be detected")
        ROI_TURN_STOP = [280, 380, 360, 410]
        middle_contours_black, hierarchy = cv2.findContours(
            img_thresh_black[
                ROI_TURN_STOP[1] : ROI_TURN_STOP[3], ROI_TURN_STOP[0] : ROI_TURN_STOP[2]
            ],
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE,
        )
        middle_area = 0
        for i in range(len(middle_contours_black)):
            cnt = middle_contours_black[i]

            area = cv2.contourArea(cnt)

            middle_area = max(area, middle_area)
        if num_pillars_r == 0:
            noRed = True

        if noRed or (tpt2red == True and num_pillars_r == 1):

            if need_space:
                Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
                Board.setPWMServoPulse(6, 1340, 100)
                time.sleep(0.2)
                need_space = False

            else:
                Board.setPWMServoPulse(1, pwm(MID_SERVO - MAX_TURN_DEGREE), 1000)
                Board.setPWMServoPulse(6, 1340, 100)
                print("turning around")
            if (
                middle_area > 80
                or (trackDir == "right" and max_orange_area > LINE_THRESHOLD)
                or (trackDir == "left" and max_blue_area > LINE_THRESHOLD)
            ):
                Board.setPWMServoPulse(6, 1500, 100)
                Board.setPWMServoPulse(1, pwm(MID_SERVO + MAX_TURN_DEGREE), 1000)
                time.sleep(0.5)
                Board.setPWMServoPulse(6, 1580, 100)
                time.sleep(4)
                Board.setPWMServoPulse(6, 1500, 100)
                time.sleep(0.1)
                Board.setPWMServoPulse(1, pwm(MID_SERVO - MAX_TURN_DEGREE), 1000)
                Board.setPWMServoPulse(6, 1340, 100)
                time.sleep(1.6)
            else:
                continue
            if trackDir == "right":
                trackDir = "left"
            else:
                trackDir = "right"
            lastLapTurnAround = False
            threeLaps = True

            if turnDir != None:
                total_turn += 1
                turnDir = None
                print("h")
    # --------------------------------------------------- Turning Logic ---------------------------------------------------
    if trackDir == None:
        if max_blue_area > max_orange_area and max_blue_area > LINE_THRESHOLD:
            trackDir = "left"
        if max_orange_area > max_blue_area and max_orange_area > LINE_THRESHOLD:
            trackDir = "right"

    elif trackDir == "right":
        if turnDir == "right" and max_blue_area > LINE_THRESHOLD:
            line_seen = True

        if (
            turnDir == "right"
            and max_blue_area < LINE_THRESHOLD
            and max_orange_area < LINE_THRESHOLD
            and (right_area_bot > 50 if not parking_algorithm else right_area > 50)
            and line_seen
        ):
            turnDir = None
            total_turn += 1
            print("done turning")
            line_seen = False

        elif max_orange_area > LINE_THRESHOLD:

            if turnDir == None:
                print("right")
                turnDir = "right"

    elif trackDir == "left":
        if turnDir == "left" and max_orange_area > LINE_THRESHOLD:
            print("sene")
            line_seen = True

        if (
            turnDir == "left"
            and max_orange_area < LINE_THRESHOLD
            and max_blue_area < LINE_THRESHOLD
            and (left_area_bot > 50 if not parking_algorithm else left_area > 50)
            and line_seen
        ):

            turnDir = None
            total_turn += 1
            print("done turning")
            line_seen = False

        elif max_blue_area > LINE_THRESHOLD:
            if turnDir == None:
                print("left")
                turnDir = "left"

    if turnDir == "left" and closest_pillar_colour == "green":
        servo_angle = MID_SERVO + MAX_TURN_DEGREE
    elif turnDir == "right" and closest_pillar_colour == "red":
        servo_angle = MID_SERVO - MAX_TURN_DEGREE

    # --------------------------------------------------- Pillar Avoidance ---------------------------------------------------

    if (
        target != None
        and not (
            left_area > 5200
            and closest_pillar_colour == "green"
            and (closest_pillar_x >= green_target or closest_pillar_area < 2000)
        )
        and not (
            right_area > 5500
            and closest_pillar_colour == "red"
            and (closest_pillar_x <= red_target or closest_pillar_area < 2000)
        )
        and not (parkingL or parkingR)
    ):
        last_difference = 0
        """
        
        if closest_pillar_colour == "green" or (
            parking_algorithm and trackDir == "right"
        ):

            if (
                closest_pillar_area > 9000
                and (closest_pillar_x) < (410 if not (parking_algorithm and trackDir == "right") else 340)
                and closest_pillar_dist < 2000
            ):
                servo_angle = MID_SERVO

                pw = pwm(servo_angle)

                Board.setPWMServoPulse(6, 1500, 100)
                Board.setPWMServoPulse(1, pw, 1000)
                time.sleep(0.3)
                Board.setPWMServoPulse(6, 1580, 100)
                time.sleep(1.5)

        elif closest_pillar_colour == "red" or (
            parking_algorithm and trackDir == "left"
        ):

            if (
                closest_pillar_area > 9000
                and (closest_pillar_x)
                > (230 if not (parking_algorithm and trackDir == "left") else 300)
                and closest_pillar_dist < 2000
            ):
                servo_angle = MID_SERVO

                pw = pwm(servo_angle)
                Board.setPWMServoPulse(6, 1500, 100)
                Board.setPWMServoPulse(1, pw, 1000)
                time.sleep(0.3)

                Board.setPWMServoPulse(6, 1580, 100)
                time.sleep(1.5)
        
        
        """
        if closest_pillar_colour == "green" and not parking_algorithm:

            if (
                closest_pillar_area > 9700
                and (closest_pillar_x) < 390
                and closest_pillar_dist < 2000
            ):
                servo_angle = MID_SERVO

                pw = pwm(servo_angle)

                Board.setPWMServoPulse(6, 1500, 100)
                Board.setPWMServoPulse(1, pw, 1000)
                time.sleep(0.3)
                Board.setPWMServoPulse(6, 1580, 100)
                time.sleep(1.5)

        elif closest_pillar_colour == "red" and not parking_algorithm:

            if (
                closest_pillar_area > 9700
                and (closest_pillar_x) > 250
                and closest_pillar_dist < 2000
            ):
                servo_angle = MID_SERVO

                pw = pwm(servo_angle)
                Board.setPWMServoPulse(6, 1500, 100)
                Board.setPWMServoPulse(1, pw, 1000)
                time.sleep(0.3)

                Board.setPWMServoPulse(6, 1580, 100)
                time.sleep(1.5)

        elif (
            parking_algorithm
            and closest_pillar_area > 9700
            and (closest_pillar_x) > 250
            and closest_pillar_dist < 2000
            and trackDir == "left"
        ):
            servo_angle = MID_SERVO

            pw = pwm(servo_angle)
            Board.setPWMServoPulse(6, 1500, 100)
            Board.setPWMServoPulse(1, pw, 1000)
            time.sleep(0.3)

            Board.setPWMServoPulse(6, 1580, 100)
            time.sleep(1.5)
        # if pillar is seen during a turn
        if turnDir != None and (num_pillars_r >= 1 or num_pillars_g >= 1):

            OBSTACLEPG = 0.03
            YAXISPG = 0.4

        # specific cases for two pillars at a turn
        elif num_pillars_g + num_pillars_r >= 2 and not parking_algorithm:

            if closest_pillar_colour == "red" and num_pillars_g >= 1:
                OBSTACLEPG = 0.001
                YAXISPG = 0.03
                print("red-green turn case")

            elif (
                closest_pillar_colour == "red"
                and num_pillars_r >= 2
                and trackDir == "right"
            ) or (
                closest_pillar_colour == "green"
                and num_pillars_g >= 2
                and trackDir == "left"
            ):
                OBSTACLEPG = 0.0018
                OBSTACLEPD = 0.08
                print("inside turn")

        error = target - closest_pillar_x

        servo_angle = (
            MID_SERVO
            + ((((error) * MAX_TURN_DEGREE) * OBSTACLEPG))
            + (error - prevError) * OBSTACLEPD
        )

        if error <= 0:
            servo_angle -= int(YAXISPG * (closest_pillar_y + h - 120))
        else:
            servo_angle += int(YAXISPG * (closest_pillar_y + h - 120))

        if turnDir == "right" and closest_pillar_colour == "red":
            servo_angle = MID_SERVO - MAX_TURN_DEGREE
        elif turnDir == "left" and closest_pillar_colour == "green":
            servo_angle = MID_SERVO + MAX_TURN_DEGREE

        if turnDir == "right" and servo_angle > MID_SERVO + (MAX_TURN_DEGREE // 4):
            servo_angle = MID_SERVO + (MAX_TURN_DEGREE // 4)
        if turnDir == "left" and servo_angle < MID_SERVO - (MAX_TURN_DEGREE // 4):
            servo_angle = MID_SERVO - (MAX_TURN_DEGREE // 4)

    # --------------------------------------------------- Wall Following ---------------------------------------------------

    elif not (parkingL or parkingR):

        if turnDir == "right" and not (left_area > 5200) and not (right_area > 5200):
            prevError = 0
            last_difference = 0

            servo_angle = MID_SERVO - MAX_TURN_DEGREE

        elif turnDir == "left" and not (right_area > 5200) and not (left_area > 5200):
            prevError = 0
            last_difference = 0
            servo_angle = MID_SERVO + MAX_TURN_DEGREE

        else:

            error = 0
            current_difference = left_area - right_area
            if last_difference != 0 and left_area != 0 and right_area != 0:
                servo_angle = MID_SERVO - (
                    current_difference * PG
                    + (current_difference - last_difference) * PD
                )
            else:
                servo_angle = MID_SERVO - (current_difference * PG)

    if total_turn >= 12:

        if action_counter >= ACTIONS_TO_STRAIGHT and need_pause:
            # Board.setPWMServoPulse(6, 1500, 100)
            # time.sleep(2)
            need_pause = False
        else:
            action_counter += 1

        if trackDir == "right":
            green_target = 480
            red_target = 480
        else:
            green_target = 160
            red_target = 160

        parking_algorithm = True
    # --------------------------------------------------- Checks if wall is too close ---------------------------------------------------

    if (middle_area > 750 or magenta_area_center > 300) and not (parkingL or parkingR):
        Board.setPWMServoPulse(6, 1500, 100)

        """
        if closest_pillar_colour == None:
            servo_angle = MID_SERVO
        
        elif closest_pillar_colour == "green" and trackDir == "right":
            servo_angle = MID_SERVO - MAX_TURN_DEGREE
        elif closest_pillar_colour == "red" and trackDir == "right":
            servo_angle = MID_SERVO + MAX_TURN_DEGREE

        elif closest_pillar_colour == "red" and trackDir == "left":
            servo_angle = MID_SERVO - MAX_TURN_DEGREE
        elif closest_pillar_colour == "green" and trackDir == "left":
            servo_angle = MID_SERVO + MAX_TURN_DEGREE
        """
        servo_angle = MID_SERVO

        pw = pwm(servo_angle)
        Board.setPWMServoPulse(1, pw, 1000)
        time.sleep(0.1)
        Board.setPWMServoPulse(6, 1580, 100)
        time.sleep(2)

    """
    
    print("OBSTACLE PG: " + str((error) * MAX_TURN_DEGREE * OBSTACLEPG))
    print("OBSTACLE PD: " + str((error - prevError) * OBSTACLEPD))

    if error <= 0:
        print("Y AXIS MULTIPLIER: " + str((-1 * YAXISPG * ((y + h - 120)))))

    else:
        print("Y AXIS MULTIPLIER: " + str((YAXISPG * y + h - 120)))
    print("TOTAL ANGLE " + str(servo_angle))
    
    
    
    
    """

    #  ---------------------------------------------- Checks if the last lap is reached and the find the direction of the appropriate last lap-----------------------------------------------
    if total_turn >= 8 and not lastLapContinue and not threeLaps:
        action_counter += 1
        if num_pillars_r == 2:
            tpt2red = True
            lastLapTurnAround = True
        elif num_pillars_r == 1:
            lastLapTurnAround = True

        elif num_pillars_g >= 1:

            lastLapContinue = True
        else:

            if prevPillar == "green":
                lastLapContinue = True
            if prevPillar == "red":

                lastLapTurnAround = True

    # --------------------------------------------------- Set variables that will be used in the next iteration ---------------------------------------------------
    last_difference = current_difference
    prevError = error
    last_target = target

    # --------------------------------------------------- Move the motors using the variables ---------------------------------------------------

    if servo_angle < MID_SERVO - MAX_TURN_DEGREE:
        servo_angle = MID_SERVO - MAX_TURN_DEGREE

    elif servo_angle > MID_SERVO + MAX_TURN_DEGREE:
        servo_angle = MID_SERVO + MAX_TURN_DEGREE

    if not (parkingR or parkingL):
        pw = pwm(servo_angle)
        Board.setPWMServoPulse(6, DC_SPEED, 100)
        Board.setPWMServoPulse(1, pw, 1000)

    # --------------------------------------------------- Debugging Functionality ---------------------------------------------------
    image = cv2.line(
        im,
        (ROI_LEFT_TOP[0], ROI_LEFT_TOP[1]),
        (ROI_LEFT_TOP[2], ROI_LEFT_TOP[1]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_LEFT_TOP[0], ROI_LEFT_TOP[1]),
        (ROI_LEFT_TOP[0], ROI_LEFT_TOP[3]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_LEFT_TOP[2], ROI_LEFT_TOP[3]),
        (ROI_LEFT_TOP[2], ROI_LEFT_TOP[1]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_LEFT_TOP[2], ROI_LEFT_TOP[3]),
        (ROI_LEFT_TOP[0], ROI_LEFT_TOP[3]),
        (0, 255, 255),
        4,
    )

    image = cv2.line(
        im,
        (ROI_RIGHT_TOP[0], ROI_RIGHT_TOP[1]),
        (ROI_RIGHT_TOP[2], ROI_RIGHT_TOP[1]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_RIGHT_TOP[0], ROI_RIGHT_TOP[1]),
        (ROI_RIGHT_TOP[0], ROI_RIGHT_TOP[3]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_RIGHT_TOP[2], ROI_RIGHT_TOP[3]),
        (ROI_RIGHT_TOP[2], ROI_RIGHT_TOP[1]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_RIGHT_TOP[2], ROI_RIGHT_TOP[3]),
        (ROI_RIGHT_TOP[0], ROI_RIGHT_TOP[3]),
        (0, 255, 255),
        4,
    )

    image = cv2.line(
        im,
        (ROI_LEFT_BOT[0], ROI_LEFT_BOT[1]),
        (ROI_LEFT_BOT[2], ROI_LEFT_BOT[1]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_LEFT_BOT[0], ROI_LEFT_BOT[1]),
        (ROI_LEFT_BOT[0], ROI_LEFT_BOT[3]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_LEFT_BOT[2], ROI_LEFT_BOT[3]),
        (ROI_LEFT_BOT[2], ROI_LEFT_BOT[1]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_LEFT_BOT[2], ROI_LEFT_BOT[3]),
        (ROI_LEFT_BOT[0], ROI_LEFT_BOT[3]),
        (0, 255, 255),
        4,
    )

    image = cv2.line(
        im,
        (ROI_RIGHT_BOT[0], ROI_RIGHT_BOT[1]),
        (ROI_RIGHT_BOT[2], ROI_RIGHT_BOT[1]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_RIGHT_BOT[0], ROI_RIGHT_BOT[1]),
        (ROI_RIGHT_BOT[0], ROI_RIGHT_BOT[3]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_RIGHT_BOT[2], ROI_RIGHT_BOT[3]),
        (ROI_RIGHT_BOT[2], ROI_RIGHT_BOT[1]),
        (0, 255, 255),
        4,
    )
    image = cv2.line(
        im,
        (ROI_RIGHT_BOT[2], ROI_RIGHT_BOT[3]),
        (ROI_RIGHT_BOT[0], ROI_RIGHT_BOT[3]),
        (0, 255, 255),
        4,
    )

    image = cv2.line(
        im,
        (ROI_MIDDLE[0], ROI_MIDDLE[1]),
        (ROI_MIDDLE[2], ROI_MIDDLE[1]),
        (0, 255, 255),
        2,
    )
    image = cv2.line(
        im,
        (ROI_MIDDLE[0], ROI_MIDDLE[1]),
        (ROI_MIDDLE[0], ROI_MIDDLE[3]),
        (0, 255, 255),
        2,
    )
    image = cv2.line(
        im,
        (ROI_MIDDLE[2], ROI_MIDDLE[3]),
        (ROI_MIDDLE[2], ROI_MIDDLE[1]),
        (0, 255, 255),
        2,
    )
    image = cv2.line(
        im,
        (ROI_MIDDLE[2], ROI_MIDDLE[3]),
        (ROI_MIDDLE[0], ROI_MIDDLE[3]),
        (0, 255, 255),
        2,
    )

    drawROI(ROI_PARKING_LEFT)
    drawROI(ROI_PARKING_RIGHT)

    image = cv2.line(im, (ROI4[0], ROI4[1]), (ROI4[2], ROI4[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI4[0], ROI4[1]), (ROI4[0], ROI4[3]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI4[2], ROI4[3]), (ROI4[2], ROI4[1]), (0, 255, 255), 4)
    image = cv2.line(im, (ROI4[2], ROI4[3]), (ROI4[0], ROI4[3]), (0, 255, 255), 4)
    if target != None:
        image = cv2.line(im, (target, 0), (target, 520), (255, 255, 0), 1)
    # display the camera
    cv2.imshow("Camera", im)
    if closest_pillar_colour == "red":
        Board.PixelColor(255, 0, 0)
    if closest_pillar_colour == "green":
        Board.PixelColor(0, 255, 0)

    if cv2.waitKey(1) == ord("q"):  #
        time.sleep(0.02)
        stop()
        print("stop")

        break


cv2.destroyAllWindows()
