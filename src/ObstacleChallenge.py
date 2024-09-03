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
    """
    Convert angle in degrees to pulse width modulation (PWM) for servo to understand

    Parameters:
    degree (int): The angle the servo should turn to.

    Returns:
    int: The equivalent pwm for the desired angle.
    """
    return round(degree * 11.1 + 500)


def stop():
    """
    Sets servo and DC motor straight and closing the OpenCV window

    Parameters:

    Returns:

    """
    Board.setPWMServoPulse(6, 1500, 100)
    Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
    Board.setPWMServoPulse(6, 1500, 100)
    cv2.destroyAllWindows()
    sys.exit()
    return 0


def findMaxContour(contours):
    """
    Find the largest contour in a list of contours

    Parameters:
    contours (list): A list of contours

    Returns:
    max_area (int): The area of the largest contour
    """

    max_area = 0
    for i in range(len(contours)):

        area = cv2.contourArea(contours[i])
        max_area = max(area, max_area)
    return max_area


Board.setPWMServoPulse(1, pwm(MID_SERVO), 1)  # turn servo to mid
Board.setPWMServoPulse(6, 1500, 100)  # arm the esc motor
time.sleep(1)

while True:

    # obstacle avoidance variables that are reset every iteration of the main loop

    # ROIs

    OBSTACLEPG = 0.0017  # proportional gain for obstacle avoidance

    OBSTACLEPD = 0.005  # derivative gain for obstacle avoidance

    YAXISPG = 0.017  # y-axis steering that is proportional to the y-axis of the pillar

    num_pillars_g = 0  # tracks how many green pillars are currently detected

    num_pillars_r = 0  # tracks how many red pillars are currently detected

    x, y, w, h = 0, 0, 0, 0  # variables for the bounding rectangle of the pillar

    closest_pillar_distance = 100000  # tracks the distance to the closest pillar (set to a large number if no pillar is detected)

    closest_pillar_x = None  # tracks the x-coordinate of the closest pillar

    closest_pillar_y = None  # tracks the y-coordinate of the closest pillar

    closest_pillar_area = None  # tracks the area of the closest pillar

    closest_pillar_colour = None  # tracks the colour of the closest pillar

    if parking_algorithm:  # if the parking algorithm is enabled
        PILLAR_SIZE = 100  # the pillar size threshold should be lower so that the pillars can be seen in advance, keeping the robot on the outside of the pillars and at the best angle to park
        OBSTACLEPG = 0.0025  # the proportional gain for obstacle avoidance should be higher as the robot only needs to stay on the outside of the pillars

    # setup camera frame
    im = picam2.capture_array()
    input = np.float32(POINTS)
    output = np.float32(
        [(0, 0), (WIDTH - 1, 0), (WIDTH - 1, HEIGHT - 1), (0, HEIGHT - 1)]
    )

    # convert to hsv
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    # --------------------------------------------------- Set The Image Thresholds ---------------------------------------------------
    # these variables store integers: either 0 (color not detected in the pixel) or 255 (color detected in the pixel)

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
    # These variables store arrays of contours using the cv2.findContours function

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

    max_magenta_area_middle = 0  # stores the area of the largest magenta contour in the middle ROI (used for detecting pillars)

    for (
        i
    ) in (
        middle_contours_magenta
    ):  # loops through all of the magenta contours in the middle ROI and finds the max
        area = cv2.contourArea(i)

        if area > max_magenta_area_middle and area > PILLAR_SIZE:

            max_magenta_area_middle = area  # finds max area

            # makes a bounding rectangle around the magenta contour
            approx = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, True), True)
            x, y, w, h = cv2.boundingRect(approx)

            # fixing the offset the coordinates using the ROI
            x += ROI_MIDDLE[0]
            y += ROI_MIDDLE[1]

            # draws bounding rectangle for debugging purposes
            image = cv2.line(im, (x, y), (x + w, y), (255, 255, 255), 1)
            image = cv2.line(im, (x, y), ((x, h + y)), (255, 255, 255), 1)
            image = cv2.line(im, (x + w, y), (x + w, y + h), (255, 255, 255), 1)
            image = cv2.line(im, (x, y + h), (x + w, y + h), (255, 255, 255), 1)

    # loops through all red colors in the middle ROI and sets them to 0 if they are contained in the bounding rectangle
    # this prevents the magenta contour from being detected as red, in case the lighting makes it unclear
    for i in range((y - 10) if y >= 10 else (0), (y + h + 10) if y + h <= 470 else 480):
        for j in range(
            (x - 50) if x >= 50 else (0), (x + w + 50) if x + w <= 590 else 640
        ):
            img_thresh_red[i][j] = 0

    # rest of the contours
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

    # define variables for max area of contours in each ROI
    left_area_top_black = 0
    left_area_bot_black = 0
    right_area_top_black = 0
    right_area_bot_black = 0
    middle_area_black = 0
    green_pillar_area = 0
    red_pillar_area = 0
    blue_line_area = 0
    orange_line_area = 0

    # store information about contours in a list for debugging
    contours_coloured = [
        (contours_red, (0, 0, 255), 1, ROI_MIDDLE),
        (contours_green, (0, 255, 0), 1, ROI_MIDDLE),
        (contours, (0, 255, 255), 1, (0, 0)),
        (contours_orange, (52, 140, 235), 0, ROI4),
        (contours_blue, (235, 67, 52), 0, ROI4),
        (left_contours_magenta, (255, 0, 255), 0, ROI_PARKING_LEFT),
        (right_contours_magenta, (255, 0, 255), 0, ROI_PARKING_RIGHT),
    ]

    # loop through contours and find the maximum area

    left_area_top_black = findMaxContour(left_top_contours_black)
    left_area_bot_black = findMaxContour(left_bot_contours_black)
    right_area_top_black = findMaxContour(right_top_contours_black)
    right_area_bot_black = findMaxContour(right_bot_contours_black)
    middle_area_black = findMaxContour(middle_contours_black)

    # combine the black areas on each side to get the total area

    right_area = right_area_bot_black + right_area_top_black
    left_area = left_area_bot_black + left_area_top_black

    # loop through contours and find the maximum area and draw them for debugging

    for i in range(len(contours_orange)):
        cnt = contours_orange[i]
        orange_line_area = max(cv2.contourArea(cnt), orange_line_area)
        cnt[:, :, 0] += ROI4[0]
        cnt[:, :, 1] += ROI4[1]
        cv2.drawContours(im, contours_orange, i, (255, 255, 0), 1)

    for i in range(len(contours_blue)):
        cnt = contours_blue[i]
        blue_line_area = max(cv2.contourArea(cnt), blue_line_area)
        cnt[:, :, 0] += ROI4[0]
        cnt[:, :, 1] += ROI4[1]

        cv2.drawContours(im, contours_blue, i, (255, 255, 0), 1)

    for i in range(len(center_contours_magenta)):
        cnt = center_contours_magenta[i]
        magenta_area_center = max(cv2.contourArea(cnt), magenta_area_center)
        cnt[:, :, 0] += ROI4[0]
        cnt[:, :, 1] += ROI4[1]

    # --------------------------------------------------- Red Pillar Detection ---------------------------------------------------

    for i in contours_red:

        area = cv2.contourArea(i)

        if (
            area > PILLAR_SIZE
        ):  # check if the pillar area is greater than the pillar size threshold

            approx = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, True), True)
            x, y, w, h = cv2.boundingRect(
                approx
            )  # make a bounding rectangle around the pillar and store the coordinates, width, and height

            pillar_distance = math.dist(
                [x + w // 2, y], [320, 480]
            )  # set the pillar distance from the top of the pillar to the center bottom of the camera view

            if pillar_distance < 500:

                # draw bounding rectangle of the pillar for debuggin gpurposes
                image = cv2.line(im, (x, y), (x + w, y), (0, 255, 255), 1)
                image = cv2.line(im, (x, y), ((x, h + y)), (0, 255, 255), 1)
                image = cv2.line(im, (x + w, y), (x + w, y + h), (0, 255, 255), 1)
                image = cv2.line(im, (x, y + h), (x + w, y + h), (0, 255, 255), 1)
                cv2.circle(im, (int(x + (w / 2)), y), 5, (255, 255, 0), 1, -1)

                num_pillars_r += 1

                if pillar_distance < closest_pillar_distance:
                    if (
                        y + h > 440 and x + (w // 2) > 240
                    ):  # using the y coordinate and x coordinate to determine if the pillar has already been passed
                        # pillar has already been passed
                        pass  # do not set as the closest pillar
                    elif (
                        y + h < 200
                    ):  # using the y coordinate of the pillar determine if the pillar is too far away to be the closest pillar

                        pass  # do not set as the closest pillar
                    else:
                        # set closest pillar variables

                        closest_pillar_distance = pillar_distance
                        closest_pillar_colour = "red"
                        closest_pillar_x = x + w // 2
                        closest_pillar_y = y  # top of the pillar
                        closest_pillar_area = h * w

    # --------------------------------------------------- Green Pillar Detection -----------------------------------------------

    for i in contours_green:
        area = cv2.contourArea(i)

        if (
            area > PILLAR_SIZE
        ):  # check if the pillar area is greater than the pillar size threshold

            approx = cv2.approxPolyDP(i, 0.01 * cv2.arcLength(i, True), True)
            x, y, w, h = cv2.boundingRect(
                approx
            )  # make a bounding rectangle around the pillar and store the coordinates, width, and height

            pillar_distance = math.dist(
                [x + w // 2, y], [320, 480]
            )  # set the pillar distance from the top of the pillar to the center bottom of the camera view

            if pillar_distance < 500:

                # draw bounding rectangle of the pillar for debugging purposes
                image = cv2.line(im, (x, y), (x + w, y), (0, 255, 255), 1)
                image = cv2.line(im, (x, y), ((x, h + y)), (0, 255, 255), 1)
                image = cv2.line(im, (x + w, y), (x + w, y + h), (0, 255, 255), 1)
                image = cv2.line(im, (x, y + h), (x + w, y + h), (0, 255, 255), 1)
                cv2.circle(im, (int(x + (w / 2)), y), 5, (255, 255, 0), 1, -1)

                num_pillars_g += 1

                if pillar_distance < closest_pillar_distance:
                    if (
                        y + h > 440 and x + (w // 2) > 400
                    ):  # using the y coordinate and x coordinate to determine if the pillar has already been passed
                        # pillar has already been passed
                        pass  # do not set as the closest pillar
                    elif (
                        y + h < 200
                    ):  # using the y coordinate of the pillar determine if the pillar is too far away to be the closest pillar
                        pass  # do not set as the closest pillar
                    else:
                        # set closest pillar variables
                        closest_pillar_distance = pillar_distance
                        closest_pillar_colour = "green"
                        closest_pillar_x = x + w // 2
                        closest_pillar_y = y  # top of the pillar
                        closest_pillar_area = h * w
    # --------------------------------------------------- Set Target ---------------------------------------------------
    # set the target based on the colour of the closest pillar
    # the robot uses the target to decide how much to turn the servo motor, depending on the distance between the target and the pillar x-coordinate
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

        parking_info = [
            [0, 0, ROI_PARKING_LEFT],
            [0, 0, ROI_PARKING_RIGHT],
            [0, 0, ROI4],
        ]

        magenta_contours = [
            left_contours_magenta,
            right_contours_magenta,
            center_contours_magenta,
        ]

        # finds the largest magenta contour in the left and right regions of interest and the y-coordinates
        for i in range(len(magenta_contours)):
            for x in range(len(magenta_contours[i])):
                cnt = magenta_contours[i][x]
                area = cv2.contourArea(cnt)

                if area > 200:

                    # get width, height, and x and y coordinates by bounding rect
                    approx = cv2.approxPolyDP(
                        cnt, 0.01 * cv2.arcLength(cnt, True), True
                    )

                    x, y, w, h = cv2.boundingRect(
                        approx
                    )  # set a bounding rectangle of the magenta contours

                    # fix the offset of the x and y coordinates based on the ROI
                    x += parking_info[i][2][0]
                    y += (
                        parking_info[i][2][1] + h
                    )  # add height for y to be the value of the bottom of the magenta contour

                    # find the largest area
                    if area > parking_info[i][0]:
                        parking_info[i][0] = area
                        parking_info[i][1] = y

            magenta_area_left = parking_info[0][
                0
            ]  # biggest magenta contour on the left ROI
            magenta_y_left = parking_info[0][1]
            magenta_area_right = parking_info[1][
                0
            ]  # biggest magenta contour on the right ROI
            magenta_y_right = parking_info[1][1]
            magenta_center_y = parking_info[2][1]

            # if the magenta contour y coordinate is low enough on the captured image and the area is greater than 150, initiate parking
            if magenta_y_left >= 534 and magenta_area_left > 150:
                if not parkingL and not parkingR:

                    parkingL = True
                    Board.setPWMServoPulse(6, 1346, 100)

                ROI4 = [
                    250,
                    250,
                    390,
                    300,
                ]  # modify the existing line-detecting ROI to become a center ROI for detecting magenta contours

            # if the magenta contour y coordinate is low enough on the captured image and the area is greater than 150, initiate parking
            if magenta_y_right >= 534 and magenta_area_right > 150:
                if not parkingL and not parkingR:

                    parkingR = True
                    Board.setPWMServoPulse(6, 1346, 100)

                ROI4 = [
                    270,
                    390,
                    370,
                    420,
                ]  # modify the existing line-detecting ROI to become a center ROI for detecting magenta contours

            if parkingR:

                # reverse at an angle if magenta contours are detected in the center ROI
                # this allows the robot to keep adjusting until no magenta contours are in the center, meaning it has parked in the parking lot
                if magenta_center_y > 380:
                    Board.setPWMServoPulse(6, 1500, 100)

                    Board.setPWMServoPulse(1, pwm(MID_SERVO + MAX_TURN_DEGREE), 1000)
                    time.sleep(0.1)

                    Board.setPWMServoPulse(6, 1580, 100)

                    time.sleep(0.4)
                    Board.setPWMServoPulse(6, 1500, 100)

                # turn right into parking lot
                else:

                    Board.setPWMServoPulse(1, pwm(MID_SERVO - MAX_TURN_DEGREE), 1000)

                    Board.setPWMServoPulse(6, 1350, 100)

            elif parkingL:
                # reverse at an angle if magenta contours are detected in the center ROI
                # this allows the robot to keep adjusting until no magenta contours are in the center, meaning it has parked in the parking lot
                if magenta_center_y > 380:

                    Board.setPWMServoPulse(6, 1500, 100)

                    Board.setPWMServoPulse(1, pwm(MID_SERVO - MAX_TURN_DEGREE), 1000)
                    time.sleep(0.1)

                    Board.setPWMServoPulse(6, 1580, 100)

                    time.sleep(0.4)
                    Board.setPWMServoPulse(6, 1500, 100)

                # turn left into parking lot
                else:

                    Board.setPWMServoPulse(1, pwm(MID_SERVO + MAX_TURN_DEGREE), 1000)

                    Board.setPWMServoPulse(6, 1350, 100)

            # if the black contour area of the wall in front is above a limit stop the robot
            if middle_area_black > 1500:
                print("stop")
                Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
                Board.setPWMServoPulse(6, 1350, 100)
                time.sleep(
                    1
                )  # continue moving the robot directly straight for a second to ensure it has fully entered the parking lot
                stop()
                break

    # --------------------------------------------------- Three Point Turn ---------------------------------------------------

    if lastLapTurnAround:

        ROI_TURN_STOP = [
            280,
            380,
            360,
            410,
        ]  # create a new ROI that detects when the robot should stop turning as it is too close to the wall

        middle_contours_black, hierarchy = (
            cv2.findContours(  # find the black contours in the middle ROI
                img_thresh_black[
                    ROI_TURN_STOP[1] : ROI_TURN_STOP[3],
                    ROI_TURN_STOP[0] : ROI_TURN_STOP[2],
                ],
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_NONE,
            )
        )

        middle_area_black = findMaxContour(
            middle_contours_black
        )  # find the largest black contour in the middle ROI

        if (
            num_pillars_r == 0
        ):  # wait until a red pillar is no longer detected to start the turn
            noRed = True

        if noRed or (
            tpt2red == True and num_pillars_r == 1
        ):  # no red or two red pillars were detected and now one has no longer been detected so begin the turn

            if (
                need_space
            ):  # the robot needs space to avoid hitting the pillar it has just passed
                Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
                Board.setPWMServoPulse(6, 1340, 100)
                time.sleep(0.2)
                need_space = False

            else:
                Board.setPWMServoPulse(1, pwm(MID_SERVO - MAX_TURN_DEGREE), 1000)
                Board.setPWMServoPulse(6, 1340, 100)
                # turn sharp right and go forward
            if (
                middle_area_black > 80
                or (trackDir == "right" and orange_line_area > LINE_THRESHOLD)
                or (trackDir == "left" and blue_line_area > LINE_THRESHOLD)
                # if a wall or the first line that signals the entrance into the first turn section is detected, reverse the robot and turn
            ):

                # reverse the robot with the servo turned sharp left
                Board.setPWMServoPulse(6, 1500, 100)
                Board.setPWMServoPulse(1, pwm(MID_SERVO + MAX_TURN_DEGREE), 1000)
                time.sleep(0.5)
                Board.setPWMServoPulse(6, 1580, 100)
                time.sleep(4)

                # stop

                Board.setPWMServoPulse(6, 1500, 100)
                time.sleep(0.1)

                # go forward and turn sharp right

                Board.setPWMServoPulse(1, pwm(MID_SERVO - MAX_TURN_DEGREE), 1000)
                Board.setPWMServoPulse(6, 1340, 100)
                time.sleep(1.6)
            else:
                continue  # if no wall is detected, keep going forward and sharp right

            # three point turn completed

            # invert the track direction
            if trackDir == "right":
                trackDir = "left"
            else:
                trackDir = "right"

            # set the lap tracking variables
            lastLapTurnAround = False
            threeLaps = True

            # if the last turn into the starting section was not fully completed, complete it

            if turnDir != None:
                total_turn += 1
                turnDir = None

    # --------------------------------------------------- Turning Logic ---------------------------------------------------
    if trackDir == None:  # if the track direction is still unknown
        if (
            blue_line_area > orange_line_area and blue_line_area > LINE_THRESHOLD
        ):  # if the blue line is detected
            trackDir = "left"  # set the track direction to left (counterclockwise)
        if (
            orange_line_area > blue_line_area and orange_line_area > LINE_THRESHOLD
        ):  # if the orange line is detected
            trackDir = "right"  # set the track direction to right (clockwise)

    elif trackDir == "right":  # if the track direction is right
        if (
            turnDir == "right" and blue_line_area > LINE_THRESHOLD
        ):  # if the robot is turning right and the blue line is detected
            line_seen = True  # set the line seen variable to true to indicate that a line has been seen and the turn can be completed if both walls are seen

        if (
            turnDir == "right"
            and blue_line_area < LINE_THRESHOLD  # no blue line detected
            and orange_line_area < LINE_THRESHOLD  # no orange line detected
            and (
                right_area_bot_black > 50
                if not parking_algorithm
                else right_area
                > 50  # only use the bottom ROI to determine if the turn can be finished to allow for more time for the robot to finish
                # if parking algorithm is enabled, the robot stays on the outside of the pillars so it should end the turn earlier instead of going to the center of the section
            )
            and line_seen  # only complete the turn if the line has been seen
        ):
            # turn completed
            turnDir = None  # reset the turn direction
            total_turn += 1  # increment the total number of turns
            print("done turning")
            line_seen = False  # reset the line seen variable

        elif (
            orange_line_area > LINE_THRESHOLD
        ):  # if the orange line is detected, start the turn

            if turnDir == None:
                print("right")
                turnDir = "right"

    elif trackDir == "left":  # if the track direction is left
        if (
            turnDir == "left" and orange_line_area > LINE_THRESHOLD
        ):  # if the robot is turning left and the orange line is detected
            print("sene")
            line_seen = True  # set the line seen variable to true to indicate that a line has been seen and the turn can be completed if both walls are seen

        if (
            turnDir == "left"
            and orange_line_area < LINE_THRESHOLD  # no orange line detected
            and blue_line_area < LINE_THRESHOLD  # no blue line detected
            and (
                left_area_bot_black > 50
                if not parking_algorithm
                else left_area
                > 50  # only use the bottom ROI to determine if the turn can be finished to allow for more time for the robot to finish
                # if parking algorithm is enabled, the robot stays on the outside of the pillars so it should end the turn earlier instead of going to the center of the section
            )
            and line_seen  # only complete the turn if the line has been seen
        ):
            # turn completed
            turnDir = None  # reset the turn direction
            total_turn += 1  # increment the total number of turns
            print("done turning")
            line_seen = False  # reset the line seen variable

        elif (
            blue_line_area > LINE_THRESHOLD
        ):  # if the blue line is detected, start the turn

            if turnDir == None:
                print("left")
                turnDir = "left"

    if (
        turnDir == "left" and closest_pillar_colour == "green"
    ):  # if the robot is turning left and the closest pillar is green this means that the robot will have to make a very sharp turn
        servo_angle = MID_SERVO + MAX_TURN_DEGREE  # set the servo angle to sharp left

    elif (
        turnDir == "right" and closest_pillar_colour == "red"
    ):  # if the robot is turning right and the closest pillar is red this means that the robot will have to make a very sharp turn
        servo_angle = MID_SERVO - MAX_TURN_DEGREE  # set the servo angle to sharp right

    # --------------------------------------------------- Pillar Avoidance ---------------------------------------------------

    if (
        target != None  # pillar detected
        and not (
            # if the robot is too close to the wall and the furthest pillar is either far away or already avoided, follow the walls instead of the pillars to center the robot
            left_area > 5200
            and closest_pillar_colour == "green"
            and (closest_pillar_x >= green_target or closest_pillar_area < 2000)
        )
        and not (
            # if the robot is too close to the wall and the furthest pillar is either far away or already avoided, follow the walls instead of the pillars to center the robot
            right_area > 5500
            and closest_pillar_colour == "red"
            and (closest_pillar_x <= red_target or closest_pillar_area < 2000)
        )
        and not (parkingL or parkingR)  # not in the process of parking
    ):
        last_difference = 0

        if closest_pillar_colour == "green" and not parking_algorithm:

            if (
                # if the pillar is too close to the robot and not on the right side, reverse the robot
                closest_pillar_area > 9700
                and (closest_pillar_x) < 390
                and closest_pillar_distance < 2000
            ):
                servo_angle = MID_SERVO  # set the servo to straight
                pw = pwm(servo_angle)
                Board.setPWMServoPulse(6, 1500, 100)  # stop the robot
                Board.setPWMServoPulse(1, pw, 1000)

                time.sleep(0.3)

                Board.setPWMServoPulse(6, 1580, 100)  # reverse
                time.sleep(1.5)

        elif closest_pillar_colour == "red" and not parking_algorithm:

            if (
                # if the pillar is too close to the robot and not on the right side, reverse the robot
                closest_pillar_area > 9700
                and (closest_pillar_x) > 250
                and closest_pillar_distance < 2000
            ):
                servo_angle = MID_SERVO  # set the servo to straight
                pw = pwm(servo_angle)
                Board.setPWMServoPulse(6, 1500, 100)  # stop the robot
                Board.setPWMServoPulse(1, pw, 1000)

                time.sleep(0.3)

                Board.setPWMServoPulse(6, 1580, 100)  # reverse
                time.sleep(1.5)

        elif (
            # if the parking algorithm is enabled, the robot stays on the outside of the pillars so it should also reverse in case it is too close and not on the right side
            parking_algorithm
            and closest_pillar_area > 9700
            and (closest_pillar_x) > 250
            and closest_pillar_distance < 2000
            and trackDir == "left"
        ):
            servo_angle = MID_SERVO  # set the servo to straight
            pw = pwm(servo_angle)

            Board.setPWMServoPulse(6, 1500, 100)  # stop the robot
            Board.setPWMServoPulse(1, pw, 1000)
            time.sleep(0.3)

            Board.setPWMServoPulse(6, 1580, 100)  # reverse
            time.sleep(1.5)

        # if pillar is seen during a turn
        if turnDir != None and (num_pillars_r >= 1 or num_pillars_g >= 1):
            # set the obstacle avoidance parameters to much higher in order to both turn and avoid the pillars
            OBSTACLEPG = 0.03
            YAXISPG = 0.4

        elif (
            num_pillars_g + num_pillars_r >= 2 and not parking_algorithm
        ):  # specific cases for two pillars at a turn as two pillars can only be seen at once when they are on opposite sides of a turning section

            if num_pillars_g == 1 and num_pillars_r == 1:
                # in this case, the robot has to go on one side of the first pillar and on the other side of the second pillar because of the alternating colours
                # the obstacle avoidance parameters are set lower so that the robot does not turn as much on the first pillar so it can be at a better position to avoid the second pillar
                OBSTACLEPG = 0.001
                YAXISPG = 0.03
                print("alternating pillars turn case")

            elif (
                # in this case, the robot has to turn in the inside of the turn section
                closest_pillar_colour == "red"
                and num_pillars_r >= 2
                and trackDir == "right"
            ) or (
                closest_pillar_colour == "green"
                and num_pillars_g >= 2
                and trackDir == "left"
            ):
                # the obstacle PD parameter is set higher to avoid the robot oscillating during the turn
                OBSTACLEPG = 0.0018
                OBSTACLEPD = 0.08
                print("inside turn")

        # calculate the error based on the target and the x coordinate of the closest pillar
        error = target - closest_pillar_x

        servo_angle = (
            MID_SERVO
            + ((((error) * MAX_TURN_DEGREE) * OBSTACLEPG))
            + (error - prevError) * OBSTACLEPD
        ) # calculate the servo angle using the error and obstacle avoidance variables

        if error <= 0: # add or subtract a value depending on the y coordinate of the closest pillar 
            #the closer the pillar is, the higher the y coordinate and the more the robot should turn to avoid it
            servo_angle -= int(YAXISPG * (closest_pillar_y + h - 120))
        else:
            servo_angle += int(YAXISPG * (closest_pillar_y + h - 120))

        # avoid the robot turning too much to the opposite direction during a turn
        # this assures that the turn is completed
        # the robot will only turn a quarter of the max turn degree in the opposite direction
        if turnDir == "right" and servo_angle > MID_SERVO + (MAX_TURN_DEGREE // 4): 
            servo_angle = MID_SERVO + (MAX_TURN_DEGREE // 4)
        if turnDir == "left" and servo_angle < MID_SERVO - (MAX_TURN_DEGREE // 4):
            servo_angle = MID_SERVO - (MAX_TURN_DEGREE // 4)

    # --------------------------------------------------- Wall Following ---------------------------------------------------

    elif not (parkingL or parkingR): # if no pillar is detected

        if turnDir == "right" and not (left_area > 5200) and not (right_area > 5200): # if one of the walls is too large, follow the walls instead to center the robot
            # reset variables
            prevError = 0
            last_difference = 0
            
            servo_angle = MID_SERVO - MAX_TURN_DEGREE # set servo to sharp right

        elif turnDir == "left" and not (right_area > 5200) and not (left_area > 5200): # if one of the walls is too large, follow the walls instead to center the robot
            # reset variables

            prevError = 0
            last_difference = 0
            servo_angle = MID_SERVO + MAX_TURN_DEGREE # set servo to sharp left

        else:

            error = 0
            current_difference = left_area - right_area # find difference between left and right areas of the wall
            if last_difference != 0 and left_area != 0 and right_area != 0:
                servo_angle = MID_SERVO - (
                    current_difference * PG
                    + (current_difference - last_difference) * PD
                )
            # use obstacle avoidance variables and the difference between the left and right areas to calculate the servo angle
            else:
                servo_angle = MID_SERVO - (current_difference * PG)

    if total_turn >= 12: # if the robot has achieved three laps

        if action_counter >= ACTIONS_TO_STRAIGHT and need_pause:  # use an action incrementing variable to enter the straight section from the turning section
            Board.setPWMServoPulse(6, 1500, 100) 
            time.sleep(2) # pause for two seconds to signal that the three laps have been completed
            need_pause = False
        else:
            action_counter += 1 # increment the action incrementing variable

        if trackDir == "right": # set the targets so that the robot stays on the outside of the pillars
            green_target = 480
            red_target = 480
        else:
            green_target = 160
            red_target = 160

        parking_algorithm = True # initiate the parking algorithm
        
        
    # --------------------------------------------------- Checks if wall is too close ---------------------------------------------------

    if (middle_area_black > 750 or magenta_area_center > 300) and not (
        parkingL or parkingR
    ):
        # if magenta or black areas get too large, it means that the robot is too close to the wall and needs to reverse
        Board.setPWMServoPulse(6, 1500, 100)
        servo_angle = MID_SERVO
        pw = pwm(servo_angle)
        Board.setPWMServoPulse(1, pw, 1000)
        time.sleep(0.1)
        Board.setPWMServoPulse(6, 1580, 100)
        time.sleep(2)

    #  ---------------------------------------------- Checks if the last lap is reached and the find the direction of the appropriate last lap-----------------------------------------------
    
    if total_turn >= 8 and not lastLapContinue and not threeLaps: 
        # check the last pillar
        if num_pillars_r == 2: # if there is two red pillars after the turn, signal to the three point turn algorithm so it can wait until only one is detected
            tpt2red = True
            lastLapTurnAround = True
            
        elif num_pillars_r == 1: # if there is one red pillar after the turn, enter the three point turn algorithm
            lastLapTurnAround = True

        elif num_pillars_g >= 1: # if the last pillar is green, continue the last lap in the same direction
            lastLapContinue = True
        else:
            # if no pillars are detected after the turn, use the previous pillar
            if prevPillar == "green":
                lastLapContinue = True
            
            if prevPillar == "red":
                lastLapTurnAround = True

    # --------------------------------------------------- Set variables that will be used in the next iteration ---------------------------------------------------
    last_difference = current_difference
    prevError = error
    last_target = target

    # --------------------------------------------------- Move the motors using the variables ---------------------------------------------------
    # make sure the servo does not overturn
    if servo_angle < MID_SERVO - MAX_TURN_DEGREE: 
        servo_angle = MID_SERVO - MAX_TURN_DEGREE

    elif servo_angle > MID_SERVO + MAX_TURN_DEGREE:
        servo_angle = MID_SERVO + MAX_TURN_DEGREE

    # set the servo angle and dc motor unless the robot is currently parking
    if not (parkingR or parkingL):
        pw = pwm(servo_angle)
        Board.setPWMServoPulse(6, DC_SPEED, 100)
        Board.setPWMServoPulse(1, pw, 1000)

    # --------------------------------------------------- Debugging Functionality ---------------------------------------------------
    
    
    # draw ROIs on display
    drawROI(ROI_LEFT_TOP)
    drawROI(ROI_RIGHT_TOP)
    drawROI(ROI_LEFT_BOT)
    drawROI(ROI_RIGHT_BOT)
    drawROI(ROI_MIDDLE)
    drawROI(ROI_PARKING_LEFT)
    drawROI(ROI_PARKING_RIGHT)
    drawROI(ROI4)
    
    # draw the target
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
