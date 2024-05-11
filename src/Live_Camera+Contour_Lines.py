import cv2
import numpy as np
import time
from picamera2 import Picamera2
# used to record the time when we processed last frame
prev_frame_time = 0

# used to record the time at which we processed current frame
new_frame_time = 0
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640,480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()
while True:
    im= picam2.capture_array()
    points = [(115,200), (525,200), (640,370), (0,370)]
    width = 640
    height = 480
    input = np.float32(points)
    output = np.float32([(0,0), (width-1,0), (width-1,height-1), (0,height-1)])
    # compute perspective matrix
    matrix = cv2.getPerspectiveTransform(input,output)
    imgPerspective = cv2.warpPerspective(im, matrix, (width,height),
    cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(0,0,0))
    #grayscaling
    imgGray = cv2.cvtColor(imgPerspective, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("gray", imgGray)
    #thresholding
    ret, imgThresh = cv2.threshold(imgGray, 110, 255, cv2.THRESH_BINARY_INV)
    contours, hierarchy = cv2.findContours(imgThresh,
        cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    print("Number of Contours found = " + str(len(contours)))
    for i in range(len(contours)):
        cnt = contours[i]
        area = cv2.contourArea(cnt)
        print(area)
        if(area > 100):
            cv2.drawContours(imgPerspective, contours, i, (0, 255, 0), 2)
            approx=cv2.approxPolyDP(cnt, 0.01*cv2.arcLength(cnt,True),True)
            x,y,w,h=cv2.boundingRect(approx)

            cv2.rectangle(imgPerspective,(x,y),(x+w,y+h),(0,0,255),2)

    cv2.imshow("Camera", imgPerspective)
    if cv2.waitKey(1)==ord("q"):#wait until key ‘q’ pressed
        break
cv2.destroyAllWindows()
