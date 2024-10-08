# libraries
import sys
import cv2  # for image processing
import numpy as np  # for image manipulation
import time

sys.path.append(
    "/home/pi/TurboPi/"
)  # navigating to where the raspberry pi libraries are located
from picamera2 import Picamera2
import HiwonderSDK.Board as Board
import RPi.GPIO as GPIO


# movement constants
MID_SERVO = 82
MAX_TURN_DEGREE = 40
DC_SPEED = 1334


# proportion constants for the servo motor angle (PID steering)
PD = 0.2
PG = 0.0035
# no integral value


# ROI constants
ROI_LEFT_BOT = [0, 300, 100, 340]
ROI_RIGHT_BOT = [540, 300, 640, 340]
ROI_LEFT_TOP = [0, 285, 40, 300]
ROI_RIGHT_TOP = [600, 285, 640, 300]
ROI4 = [270, 340, 370, 380]


# color threshold constants (in HSV)
# LOWER_BLUE = np.array([104, 65, 70])
# UPPER_BLUE = np.array([132, 205, 185])
# LOWER_ORANGE1 = np.array([155, 59, 70])
# UPPER_ORANGE1 = np.array([180, 255, 255])
# LOWER_ORANGE2 = np.array([0, 59, 70])
# UPPER_ORANGE2 = np.array([8, 255, 255])

LOWER_ORANGE1 = np.array([180, 100, 100])
UPPER_ORANGE1 = np.array([180, 255, 255])
LOWER_ORANGE2 = np.array([0, 80, 80])
UPPER_ORANGE2 = np.array([20, 255, 255])

LOWER_BLUE = np.array([101, 20, 30])
UPPER_BLUE = np.array([125, 255, 255])

LOWER_BLACK_THRESHOLD = np.array([0, 0, 0])
UPPER_BLACK_THRESHOLD = np.array([180, 255, 80])


# camera settings
WIDTH = 640
HEIGHT = 480
POINTS = [(115, 200), (525, 200), (640, 370), (0, 370)]


# limiting constants
MAX_TURNS = 12
ACTIONS_TO_STRAIGHT = 120
WALL_THRESHOLD = 50
NO_WALL_THRESHOLD = 50
TURN_ITER_LIMIT = 30
LINE_THRESHOLD = 30


# incrementing variables
action_counter = 0
turn_counter = 0
turn_length_counter = 0


# dynamic variables
track_dir = "none"
done_turning = False
turn_dir = "none"
last_difference = 0
current_difference = 0


# initialize servo angle variable
servo_angle = 0


# initialization of camera global variables
im = None
image = None

# activate buzzer, showing that the program has started
Board.setBuzzer(0)  # turn off buzzer
Board.setBuzzer(1)  # turn on buzzer
time.sleep(0.1)
Board.setBuzzer(0)  # turn off buzzer

# wait for button press
key2_pin = 16
GPIO.setmode(GPIO.BOARD)
GPIO.setup(key2_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
while GPIO.input(key2_pin) == GPIO.HIGH:
    pass


# camera setup
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.set_controls(
    {"Saturation": 1.4, "Brightness": 0.05}
)  # adjust brightness and saturation of the image
picam2.start()


# utility function for pwm angle calculation
def pwm(degree):  # angle must be adjusted to pwm angle for servo
    """
    Convert angle in degrees to pulse width modulation (PWM) for servo to understand

    Parameters:
    degree (int): The angle the servo should turn to.

    Returns:
    int: The equivalent pwm for the desired angle.
    """

    return round(degree * 11.1 + 500)


# utility function for drawing ROI
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


# utility function for stopping the robot
def stop():
    """
    Sets servo and DC motor straight and closing the OpenCV window

    Parameters:

    Returns:

    """

    time.sleep(0.02)
    Board.setPWMServoPulse(6, 1500, 100)
    Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
    time.sleep(0.05)
    Board.setPWMServoPulse(6, 1500, 100)
    cv2.destroyAllWindows()


# setting servo angle to straight and the motor to stationary
Board.setPWMServoPulse(1, pwm(MID_SERVO), 10)  # turn servo to mid
Board.setPWMServoPulse(6, 1500, 100)  # arm the esc motor
time.sleep(0.5)


# main loop
while True:

    # exit if button is pressed
    if GPIO.input(key2_pin) != GPIO.HIGH:
        stop()
        break

    # setup camera frame
    im = picam2.capture_array()
    input = np.float32(POINTS)
    output = np.float32(
        [(0, 0), (WIDTH - 1, 0), (WIDTH - 1, HEIGHT - 1), (0, HEIGHT - 1)]
    )

    # convert to hsv
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    # find black thresholds/mask and store them in a variable
    img_thresh = cv2.inRange(img_hsv, LOWER_BLACK_THRESHOLD, UPPER_BLACK_THRESHOLD)

    # find black contours in left and right regions
    left_contours_top, hierarchy = cv2.findContours(
        img_thresh[
            ROI_LEFT_TOP[1] : ROI_LEFT_TOP[3], ROI_LEFT_TOP[0] : ROI_LEFT_TOP[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    right_contours_top, hierarchy = cv2.findContours(
        img_thresh[
            ROI_RIGHT_TOP[1] : ROI_RIGHT_TOP[3], ROI_RIGHT_TOP[0] : ROI_RIGHT_TOP[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    left_contours_bot, hierarchy = cv2.findContours(
        img_thresh[
            ROI_LEFT_BOT[1] : ROI_LEFT_BOT[3], ROI_LEFT_BOT[0] : ROI_LEFT_BOT[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    right_contours_bot, hierarchy = cv2.findContours(
        img_thresh[
            ROI_RIGHT_BOT[1] : ROI_RIGHT_BOT[3], ROI_RIGHT_BOT[0] : ROI_RIGHT_BOT[2]
        ],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

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

    # combine areas on each side
    right_area = right_area_bot + right_area_top
    left_area = left_area_bot + left_area_top

    # find blue thresholds/mask and store them in a variable
    b_mask = cv2.inRange(img_hsv, LOWER_BLUE, UPPER_BLUE)

    # find blue contours to detect the lines on the mat
    contours_blue = cv2.findContours(
        b_mask[ROI4[1] : ROI4[3], ROI4[0] : ROI4[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]

    # find orange thresholds/mask and store them in a variable
    o_mask = cv2.bitwise_or(
        cv2.inRange(img_hsv, LOWER_ORANGE1, UPPER_ORANGE1),
        cv2.inRange(img_hsv, LOWER_ORANGE2, UPPER_ORANGE2),
    )

    # find orange contours to detect the lines on the mat
    contours_orange = cv2.findContours(
        o_mask[ROI4[1] : ROI4[3], ROI4[0] : ROI4[2]],
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )[-2]

    # iterate through the blue and orange contours and store the maximum area
    max_blue_area = 0
    max_orange_area = 0

    for i in range(len(contours_orange)):
        cnt = contours_orange[i]
        max_orange_area = max(cv2.contourArea(cnt), max_orange_area)
        cnt[:, :, 0] += ROI4[0]  # Add X offset
        cnt[:, :, 1] += ROI4[1]  # Add Y offset
        cv2.drawContours(
            im, contours_orange, i, (255, 255, 0), 1
        )  # draw the contours for debug utility
    for i in range(len(contours_blue)):
        cnt = contours_blue[i]
        max_blue_area = max(cv2.contourArea(cnt), max_blue_area)
        cnt[:, :, 0] += ROI4[0]  # Add X offset
        cnt[:, :, 1] += ROI4[1]  # Add Y offset
        cv2.drawContours(
            im, contours_blue, i, (255, 255, 0), 1
        )  # draw the contours for debug utility
    if track_dir == "none":  # find which direction the track is going
        if (
            max_blue_area > max_orange_area and max_blue_area > LINE_THRESHOLD
        ):  # if blue is first seen, the track direction is left (counter clockwise)
            track_dir = "left"
        if (
            max_orange_area > max_blue_area and max_orange_area > LINE_THRESHOLD
        ):  # if orange is first seen, the track direction is right (clockwise)
            track_dir = "right"
    if track_dir == "right":
        if (
            turn_dir
            == "right"  # if the robot is in a sharp turn but sees the opposite colour, the turn is stopped
            and max_blue_area > LINE_THRESHOLD
            and max_orange_area < LINE_THRESHOLD
        ):
            done_turning = True
            turn_dir = "none"
        elif max_orange_area > LINE_THRESHOLD:  # signals that a turn must occur
            # if the turn direction hasn't been changed yet change the turn direction to right

            if turn_dir == "none":
                turn_counter += 1
                turn_dir = "right"
    elif track_dir == "left":
        if (
            turn_dir
            == "left"  # if the robot is in a sharp turn but sees the opposite colour, the turn is stopped
            and max_orange_area > LINE_THRESHOLD
            and max_blue_area < LINE_THRESHOLD
        ):
            done_turning = True
            turn_dir = "none"
        elif max_blue_area > LINE_THRESHOLD:  # signals that a turn must occur
            # if the turn direction hasn't been changed yet change the turn direction to left

            if turn_dir == "none":
                turn_counter += 1
                turn_dir = "left"
    if (
        done_turning
    ):  # if the robot has just completed a turn, make it continue the turn to assure that it sees the two parallel walls
        turn_length_counter += 1

        if (
            turn_length_counter > TURN_ITER_LIMIT
        ):  # turn for the required amount of time
            turn_length_counter = 0
            done_turning = False
    else:

        if turn_dir == "right":  # calculate the servo angle for the current turn
            servo_angle = MID_SERVO - MAX_TURN_DEGREE
        elif turn_dir == "left":  # calculate the servo angle for the current turn
            servo_angle = MID_SERVO + MAX_TURN_DEGREE
        else:
            turn_length_counter = 0  # reset the turn length counter
            if (
                left_area < WALL_THRESHOLD
            ):  # if the left area is below the threshold, set the servo angle to max for a sharp left turn
                servo_angle = MID_SERVO + MAX_TURN_DEGREE
            elif (
                right_area < WALL_THRESHOLD
            ):  # if the right area is below the threshold, set the servo angle to max for a sharp right turn
                servo_angle = MID_SERVO - MAX_TURN_DEGREE
            else:
                # if in the straight section, calculate the current_difference between the contours in the left and right area

                current_difference = left_area - right_area

                # calculate steering amount using preportional-derivative steering
                # multiply the current_difference by a constant variable and add the projected error multiplied by another constant
                # this method gives stable and smooth steering

                servo_angle = MID_SERVO - (
                    current_difference * PG
                    + (current_difference - last_difference) * PD
                )
    if (
        turn_counter == MAX_TURNS and not done_turning
    ):  # if the total turns has surpassed the amount required, increment the action counter by 1
        action_counter += 1  # this is to ensure that the robot stops in the middle of the straight section

    # set the last current_difference equal to the current current_difference for derivative steering
    last_difference = current_difference

    # if the steering variable is higher than the max turn degree for the servo, set it to the max turn degree
    # move the motors using the variables
    pw = pwm(servo_angle)
    Board.setPWMServoPulse(6, DC_SPEED, 100)
    Board.setPWMServoPulse(1, pw, 1000)

    # draw the ROIs
    drawROI(ROI_LEFT_TOP)
    drawROI(ROI_RIGHT_TOP)
    drawROI(ROI_LEFT_BOT)
    drawROI(ROI_RIGHT_BOT)
    drawROI(ROI4)

    # display the camera
    if len(sys.argv) > 1 and sys.argv[1] == "Debug":
        cv2.imshow("Camera", im)

    # if the number of actions to the straight section has been met, stop the car
    if cv2.waitKey(1) == ord("q") or action_counter >= ACTIONS_TO_STRAIGHT:
        stop()
        break

cv2.destroyAllWindows()
