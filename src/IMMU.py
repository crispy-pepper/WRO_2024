
import time
import math
import sys
sys.path.append('/home/pi/BerryIMU/python-BerryIMU-gyro-accel-compass/')
import IMU
import datetime
import os
import HiwonderSDK.Board as Board

def pwm(degree):
	return round(degree*11.1 + 500)

RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
AA =  0.40 

magXmin = 559
magYmin = -1157
magZmin = -3001
magXmax = 2039
magYmax = 222
magZmax = -2575

gyroXangle = 0.0
gyroYangle = 0.0
gyroZangle = 0.0
CFangleX = 0.0
CFangleY = 0.0


targetHeading = 70
state = "GyroStraight" # or "Turn"
startTime = time.time()
a = datetime.datetime.now()
#main loop
while True:
    b = datetime.datetime.now() - a
    a = datetime.datetime.now()
    LP = b.microseconds/(1000000*1.0)
    outputString = "Loop Time %5.2f " % ( LP )

    ACCx = IMU.readACCx()
    ACCy = IMU.readACCy()
    ACCz = IMU.readACCz()
    GYRx = IMU.readGYRx()
    GYRy = IMU.readGYRy()
    GYRz = IMU.readGYRz()
    MAGx = IMU.readMAGx()
    MAGy = IMU.readMAGy()
    MAGz = IMU.readMAGz()

    #Apply compass calibration
    MAGx -= (magXmin + magXmax) /2
    MAGy -= (magYmin + magYmax) /2
    MAGz -= (magZmin + magZmax) /2


    #Convert Gyro raw to degrees per second
    rate_gyr_x =  GYRx * G_GAIN
    rate_gyr_y =  GYRy * G_GAIN
    rate_gyr_z =  GYRz * G_GAIN


    #Calculate the angles from the gyro.
    gyroXangle+=rate_gyr_x*LP
    gyroYangle+=rate_gyr_y*LP
    gyroZangle+=rate_gyr_z*LP


    #Convert Accelerometer values to degrees
    AccXangle =  (math.atan2(ACCy,ACCz)*RAD_TO_DEG)
    AccYangle =  (math.atan2(ACCz,ACCx)+M_PI)*RAD_TO_DEG

    #convert the values to -180 and +180
    if AccYangle > 90:
        AccYangle -= 270.0
    else:
        AccYangle += 90.0



    #Complementary filter used to combine the accelerometer and gyro values.
    CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
    CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle



    #Calculate heading
    heading = 180 * math.atan2(MAGy,MAGx)/M_PI

    #Only have our heading between 0 and 360
    if heading < 0:
        heading += 360

    ####################################################################
    ###################Tilt compensated heading#########################
    ####################################################################
    #Normalize accelerometer raw values.
    accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
    accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)


    #Calculate pitch and roll
    pitch = math.asin(accXnorm)
    roll = -math.asin(accYnorm/math.cos(pitch))


    #Calculate the new tilt compensated values
    #The compass and accelerometer are orientated differently on the the BerryIMUv1, v2 and v3.
    #This needs to be taken into consideration when performing the calculations

    #X compensation
    if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
        magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
    else:                                                                #LSM9DS1
        magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)

    #Y compensation
    if(IMU.BerryIMUversion == 1 or IMU.BerryIMUversion == 3):            #LSM9DS0 and (LSM6DSL & LIS2MDL)
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
    else:                                                                #LSM9DS1
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)




    #Calculate tilt compensated heading
    tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/M_PI

    if tiltCompensatedHeading < 0:
        tiltCompensatedHeading += 360
    #get &#39;heading&#39; and &#39;tiltCompensatedHeading&#39; here
    if state == "GyroStraight": 
        error = heading - targetHeading
        correction = 1.3 * error + 2 * (error - prevError)
        angle = 87 + correction
        if angle < 50:
            angle = 50
        if angle < 130:
            angle = 130
 
        print("heading: ", heading)
        print("tiltCompensatedHeading: ", tiltCompensatedHeading)
        print("angle: ", angle)
        Board.setPWMServoPulse(1, pwm(angle), 1000)

        prevError = error
        if time.time() - startTime < 10:
            if targetHeading == 70:
                print("Start turning")
                state = "Turn"
                Board.setPWMServoPulse(1, pwm(130), 1000)
            else:
                time.sleep(0.02)
                Board.setPWMServoPulse(6, 1500, 100) 
                Board.setPWMServoPulse(1, pwm(MID_SERVO), 1000)
                print("stop")
                break
    else:
        print("heading: ", heading)
        print("tiltCompensatedHeading: ", tiltCompensatedHeading)
        if heading <= -10:
            print("Stop turning")
            state = "GyroStraight"
            targetHeading = -14
            startTime = time.time()
            Board.setPWMServoPulse(1, pwm(87), 1000)
            error = 0
            prevError = 0

    time.sleep(0.01)