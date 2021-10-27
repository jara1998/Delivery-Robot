import ped_det
import cv2
import RPi.GPIO as GPIO
import time
import navigation as navi

import sys
import math
import IMU
import datetime
import os

from rplidar import RPLidar
import numpy as np
import multiprocessing
import threading




#Kalman filter variables
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = 0.0
x_bias = 0.0
XP_00 = 0.0
XP_01 = 0.0
XP_10 = 0.0
XP_11 = 0.0
YP_00 = 0.0
YP_01 = 0.0
YP_10 = 0.0
YP_11 = 0.0
KFangleX = 0.0
KFangleY = 0.0

danger = False
stop_thread = False


def kalmanFilterY ( accAngle, gyroRate, DT):
    y=0.0
    S=0.0

    global KFangleY
    global Q_angle
    global Q_gyro
    global y_bias
    global YP_00
    global YP_01
    global YP_10
    global YP_11

    KFangleY = KFangleY + DT * (gyroRate - y_bias)

    YP_00 = YP_00 + ( - DT * (YP_10 + YP_01) + Q_angle * DT )
    YP_01 = YP_01 + ( - DT * YP_11 )
    YP_10 = YP_10 + ( - DT * YP_11 )
    YP_11 = YP_11 + ( + Q_gyro * DT )

    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0 = YP_00 / S
    K_1 = YP_10 / S

    KFangleY = KFangleY + ( K_0 * y )
    y_bias = y_bias + ( K_1 * y )

    YP_00 = YP_00 - ( K_0 * YP_00 )
    YP_01 = YP_01 - ( K_0 * YP_01 )
    YP_10 = YP_10 - ( K_1 * YP_00 )
    YP_11 = YP_11 - ( K_1 * YP_01 )

    return KFangleY

def kalmanFilterX ( accAngle, gyroRate, DT):
    x=0.0
    S=0.0

    global KFangleX
    global Q_angle
    global Q_gyro
    global x_bias
    global XP_00
    global XP_01
    global XP_10
    global XP_11


    KFangleX = KFangleX + DT * (gyroRate - x_bias)

    XP_00 = XP_00 + ( - DT * (XP_10 + XP_01) + Q_angle * DT )
    XP_01 = XP_01 + ( - DT * XP_11 )
    XP_10 = XP_10 + ( - DT * XP_11 )
    XP_11 = XP_11 + ( + Q_gyro * DT )

    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0 = XP_00 / S
    K_1 = XP_10 / S

    KFangleX = KFangleX + ( K_0 * x )
    x_bias = x_bias + ( K_1 * x )

    XP_00 = XP_00 - ( K_0 * XP_00 )
    XP_01 = XP_01 - ( K_0 * XP_01 )
    XP_10 = XP_10 - ( K_1 * XP_00 )
    XP_11 = XP_11 - ( K_1 * XP_01 )

    return KFangleX


def gpio_init():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(10, GPIO.OUT)
    GPIO.setup(9, GPIO.OUT)
    GPIO.setup(11, GPIO.OUT)
    
    GPIO.output(10, GPIO.LOW)
    GPIO.output(9, GPIO.LOW)
    GPIO.output(11, GPIO.LOW)

def forward():
    GPIO.output(11, GPIO.LOW)
    GPIO.output(9, GPIO.LOW)
    GPIO.output(10, GPIO.HIGH)
    
    #print("gpio input check:", GPIO.input(10))
    print("---------------------forward")
    
    
def left():
    GPIO.output(11, GPIO.LOW)
    GPIO.output(10, GPIO.LOW)
    GPIO.output(9, GPIO.HIGH)
    print("---------------------left")
   
def right():
    GPIO.output(10, GPIO.LOW)
    GPIO.output(9, GPIO.LOW)
    GPIO.output(11, GPIO.HIGH)
    print("---------------------right")

def stop():
    GPIO.output(10, GPIO.LOW)
    GPIO.output(9, GPIO.LOW)
    GPIO.output(11, GPIO.LOW)
    print("---------------------stop")
    
heading = -999

def read_heading_thread ():

    RAD_TO_DEG = 57.29578
    M_PI = 3.14159265358979323846
    G_GAIN = 0.070          # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
    AA =  0.40              # Complementary filter constant
    MAG_LPF_FACTOR = 0.4    # Low pass filter constant magnetometer
    ACC_LPF_FACTOR = 0.4    # Low pass filter constant for accelerometer
    ACC_MEDIANTABLESIZE = 9         # Median filter table size for accelerometer. Higher = smoother but a longer delay
    MAG_MEDIANTABLESIZE = 9         # Median filter table size for magnetometer. Higher = smoother but a longer delay
    DECLINATION = 15.288


    magXmin = -880
    magYmin = -2161
    magZmin = -2283
    magXmax = 898
    magYmax = -258
    magZmax = -1785



    gyroXangle = 0.0
    gyroYangle = 0.0
    gyroZangle = 0.0
    CFangleX = 0.0
    CFangleY = 0.0
    CFangleXFiltered = 0.0
    CFangleYFiltered = 0.0
    kalmanX = 0.0
    kalmanY = 0.0
    oldXMagRawValue = 0
    oldYMagRawValue = 0
    oldZMagRawValue = 0
    oldXAccRawValue = 0
    oldYAccRawValue = 0
    oldZAccRawValue = 0

    a = datetime.datetime.now()



    #Setup the tables for the mdeian filter. Fill them all with '1' so we dont get devide by zero error
    acc_medianTable1X = [1] * ACC_MEDIANTABLESIZE
    acc_medianTable1Y = [1] * ACC_MEDIANTABLESIZE
    acc_medianTable1Z = [1] * ACC_MEDIANTABLESIZE
    acc_medianTable2X = [1] * ACC_MEDIANTABLESIZE
    acc_medianTable2Y = [1] * ACC_MEDIANTABLESIZE
    acc_medianTable2Z = [1] * ACC_MEDIANTABLESIZE
    mag_medianTable1X = [1] * MAG_MEDIANTABLESIZE
    mag_medianTable1Y = [1] * MAG_MEDIANTABLESIZE
    mag_medianTable1Z = [1] * MAG_MEDIANTABLESIZE
    mag_medianTable2X = [1] * MAG_MEDIANTABLESIZE
    mag_medianTable2Y = [1] * MAG_MEDIANTABLESIZE
    mag_medianTable2Z = [1] * MAG_MEDIANTABLESIZE

    IMU.detectIMU()     #Detect if BerryIMU is connected.
    if(IMU.BerryIMUversion == 99):
        print(" No BerryIMU found... exiting ")
        sys.exit()
    IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass
    global stop_thread
    global heading
    while True:
        #Read the accelerometer,gyroscope and magnetometer values
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


        ##Calculate loop Period(LP). How long between Gyro Reads
        b = datetime.datetime.now() - a
        a = datetime.datetime.now()
        LP = b.microseconds/(1000000*1.0)
        outputString = "Loop Time %5.2f " % ( LP )



        ###############################################
        #### Apply low pass filter ####
        ###############################################
        MAGx =  MAGx  * MAG_LPF_FACTOR + oldXMagRawValue*(1 - MAG_LPF_FACTOR);
        MAGy =  MAGy  * MAG_LPF_FACTOR + oldYMagRawValue*(1 - MAG_LPF_FACTOR);
        MAGz =  MAGz  * MAG_LPF_FACTOR + oldZMagRawValue*(1 - MAG_LPF_FACTOR);
        ACCx =  ACCx  * ACC_LPF_FACTOR + oldXAccRawValue*(1 - ACC_LPF_FACTOR);
        ACCy =  ACCy  * ACC_LPF_FACTOR + oldYAccRawValue*(1 - ACC_LPF_FACTOR);
        ACCz =  ACCz  * ACC_LPF_FACTOR + oldZAccRawValue*(1 - ACC_LPF_FACTOR);

        oldXMagRawValue = MAGx
        oldYMagRawValue = MAGy
        oldZMagRawValue = MAGz
        oldXAccRawValue = ACCx
        oldYAccRawValue = ACCy
        oldZAccRawValue = ACCz

        #########################################
        #### Median filter for accelerometer ####
        #########################################
        # cycle the table
        for x in range (ACC_MEDIANTABLESIZE-1,0,-1 ):
            acc_medianTable1X[x] = acc_medianTable1X[x-1]
            acc_medianTable1Y[x] = acc_medianTable1Y[x-1]
            acc_medianTable1Z[x] = acc_medianTable1Z[x-1]

        # Insert the lates values
        acc_medianTable1X[0] = ACCx
        acc_medianTable1Y[0] = ACCy
        acc_medianTable1Z[0] = ACCz

        # Copy the tables
        acc_medianTable2X = acc_medianTable1X[:]
        acc_medianTable2Y = acc_medianTable1Y[:]
        acc_medianTable2Z = acc_medianTable1Z[:]

        # Sort table 2
        acc_medianTable2X.sort()
        acc_medianTable2Y.sort()
        acc_medianTable2Z.sort()

        # The middle value is the value we are interested in
        ACCx = acc_medianTable2X[int(ACC_MEDIANTABLESIZE/2)];
        ACCy = acc_medianTable2Y[int(ACC_MEDIANTABLESIZE/2)];
        ACCz = acc_medianTable2Z[int(ACC_MEDIANTABLESIZE/2)];



        #########################################
        #### Median filter for magnetometer ####
        #########################################
        # cycle the table
        for x in range (MAG_MEDIANTABLESIZE-1,0,-1 ):
            mag_medianTable1X[x] = mag_medianTable1X[x-1]
            mag_medianTable1Y[x] = mag_medianTable1Y[x-1]
            mag_medianTable1Z[x] = mag_medianTable1Z[x-1]

        # Insert the latest values
        mag_medianTable1X[0] = MAGx
        mag_medianTable1Y[0] = MAGy
        mag_medianTable1Z[0] = MAGz

        # Copy the tables
        mag_medianTable2X = mag_medianTable1X[:]
        mag_medianTable2Y = mag_medianTable1Y[:]
        mag_medianTable2Z = mag_medianTable1Z[:]

        # Sort table 2
        mag_medianTable2X.sort()
        mag_medianTable2Y.sort()
        mag_medianTable2Z.sort()

        # The middle value is the value we are interested in
        MAGx = mag_medianTable2X[int(MAG_MEDIANTABLESIZE/2)];
        MAGy = mag_medianTable2Y[int(MAG_MEDIANTABLESIZE/2)];
        MAGz = mag_medianTable2Z[int(MAG_MEDIANTABLESIZE/2)];



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


        #Change the rotation value of the accelerometer to -/+ 180 and
        #move the Y axis '0' point to up.  This makes it easier to read.
        if AccYangle > 90:
            AccYangle -= 270.0
        else:
            AccYangle += 90.0



        #Complementary filter used to combine the accelerometer and gyro values.
        CFangleX=AA*(CFangleX+rate_gyr_x*LP) +(1 - AA) * AccXangle
        CFangleY=AA*(CFangleY+rate_gyr_y*LP) +(1 - AA) * AccYangle

        #Kalman filter used to combine the accelerometer and gyro values.
        kalmanY = kalmanFilterY(AccYangle, rate_gyr_y,LP)
        kalmanX = kalmanFilterX(AccXangle, rate_gyr_x,LP)

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

        heading = (tiltCompensatedHeading + DECLINATION) % 360
        ##################### END Tilt Compensation ########################


        #if 0:                       #Change to '0' to stop showing the angles from the accelerometer
        #    outputString += "#  ACX Ang %5.2f ACY Ang %5.2f  #  " % (AccXangle, AccYangle)

        #if 0:                       #Change to '0' to stop  showing the angles from the gyro
        #    outputString +="\t# GRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f # " % (gyroXangle,gyroYangle,gyroZangle)

        #if 0:                       #Change to '0' to stop  showing the angles from the complementary filter
         #   outputString +="\t#  CFangleX Angle %5.2f   CFangleY Angle %5.2f  #" % (CFangleX,CFangleY)

        #if 1:                       #Change to '0' to stop  showing the heading
        #    outputString +="\t# HEADING %5.2f  CompensatedHeading %5.2f #" % (heading,(tiltCompensatedHeading + DECLINATION)%359)

        #if 1:                       #Change to '0' to stop  showing the angles from the Kalman filter
         #   outputString +="# kalmanX %5.2f   kalmanY %5.2f #" % (kalmanX,kalmanY)

        #print(outputString)

        #slow program down a bit, makes the output more readable
        time.sleep(0.03)
        if stop_thread:
            break
        
    
    
    
PORT_NAME = '/dev/ttyUSB0'
lidar = RPLidar(PORT_NAME)

def lidar_check():
    global danger
    global stop_thread
    global lidar
    for scan in lidar.iter_scans():
        if stop_thread:
            break
        #print ("Quality:", scan[1][0])
        #print ("Angle: ", scan[1][1], 'degree')
        #print ("Distance:", scan[1][2],'mm')
        size = len(scan)
        #print(check_range)
        danger = False
        for data in scan:
            if (data[1] >= 150) and (data[1] <= 210):
                # distance less than 500 mm
                if (data[2] <= 750):
                    danger = True
                    break
        
def test_process(des):
    global danger
    while True:
        print("receiver: ", danger)
        time.sleep(1)
    
    
    
    
    
## ----------------COR detection---------------- ##

def padestrain_detect(img_curr,show_res):
    # Initializing the HOG person
    # detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # Reading the Image
    #image = cv2.imread('ex.jpg')
    image = img_curr   
    # Resizing the Image
    image = imutils.resize(image,
                           width=min(400, image.shape[1]))

    # Detecting all the regions in the 
    # Image that has a pedestrians inside it
    (regions, _) = hog.detectMultiScale(image, 
                                        winStride=(4, 4),
                                        padding=(4, 4),
                                        scale=1.05)
   
    # Drawing the regions in the Image
    for (x, y, w, h) in regions:
        if show_res:
            cv2.rectangle(image, (x, y), 
                          (x + w, y + h), 
                          (0, 0, 255), 2) 
        if ((w > 40) & (h >140)):
            print("people is coming")
            if show_res:
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(image,'people is coming',(10,20), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)
    return image




# print the image with given title using plt
def printImg(Img,title,color = 'viridis'):
    plt.title(title)
    plt.imshow(Img.astype(np.uint8)[:,:,::-1],cmap = color)
    plt.xticks([]), plt.yticks([]) 
    plt.show()
# open image with given title using cv
def openImgBox(Img,title):
    cv2.imshow(title,Img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def shrink(img):
    hight,width,channel = img.shape
    img = cv2.resize(img,(int(width/7),int(hight/7)), interpolation =cv2.INTER_AREA)
    #printImg(img,"img")

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    #img = cv2.blur(img,(5,5))
    return img

def customized_k(img):
    #img = cv2.imread('road_jara.jpg')
    #printImg(img,"img")

    #printImg(img,'img')

    Z = img.reshape((-1,3))

    # convert to np.float32
    Z = np.float32(Z)

    # define criteria, number of clusters(K) and apply kmeans()
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 5
    ret,label,center=cv2.kmeans(Z,K,None,criteria,10,cv2.KMEANS_RANDOM_CENTERS)
    center = np.uint8(center)
    res = center[label.flatten()]
    res2 = res.reshape((img.shape))
    return res2

def segment(res2):
    height,width,channel = res2.shape
    center_pix = res2[int(3*height/4),int(width/2)]
    new_res = np.zeros((height,width,channel))
    for i in range(height):
        for j in range(width):
            if res2[i,j,0] == center_pix[0]:
                new_res[i,j] = [0,255,255]
    #start_point_ori = [int(width/2),int(3*height/4)]
    #end_point_ori = start_point_ori
    #printImg(new_res,"segmented")
    return new_res

def left_right(img,buffer):
    left = 0
    right = 0
    height,width,channel = img.shape
    for i in range(int(height/2), height):
        for j in range(width):
            if img[i,j,1] == [255]:
                if j < (int(width/2)):
                    left = left +1
                else:
                    right = right +1
    #print(str(left) + ',' + str(right))
    if ((left - right) > buffer):
        #print('go left')
        return 2 # should turn right
    elif ((right - left) > buffer):
        #print ('go right')
        return 1 # should turn left
    else:
        #print("good")
        return 0
    

def get_arrow(step,img):
    new_res = img
    height,width,channel = img.shape
    start_point = [int(width/2),int(3*height/4)]
    end_point = start_point
    curr_hi = int(3*height/4)
    curr_wi = int(width/2)
    rang = int((int(3*height/4) - int(height/3))/step)
    for i in range(37):
        curr_hi = curr_hi - step
        if new_res[curr_hi,curr_wi,1] == 255.0:
            end_point[1] = curr_hi 
        elif new_res[curr_hi,curr_wi + 1,1] == 255.0:
            end_point[0] = curr_wi + 1
            curr_wi = curr_wi + 1
            #make turn right
        elif new_res[curr_hi,curr_wi - 1,1] == 255.0:
            end_point[0] = curr_wi - 1
            curr_wi = curr_wi - 1
            #make turn left
        elif new_res[curr_hi,curr_wi + 3,1] == 255.0:
            end_point[0] = curr_wi + 3
            curr_wi = curr_wi + 3
        elif new_res[curr_hi,curr_wi - 3,1] == 255.0:
            end_point[0] = curr_wi - 3
            curr_wi = curr_wi - 3
        elif new_res[curr_hi,curr_wi + 5,1] == 255.0:
            end_point[0] = curr_wi + 5
            curr_wi = curr_wi + 5
        elif new_res[curr_hi,curr_wi - 5,1] == 255.0:
            end_point[0] = curr_wi - 5
            curr_wi = curr_wi - 5
    return end_point

def rdDetect(img):
    #printImg(img,'original image')
    start_time = time.time()
    res0 = shrink(img)
    #printImg(res0,'shrinked image')
    res1 = customized_k(res0)
    #printImg(res1,'K-means segmentation')
    rea = segment(res1)
    LR_res = left_right(rea,80)
    #printImg(rea,'segmented')
    end_point = get_arrow(1,rea)
    height,width,channel = rea.shape
    start_point = (int(width/2),int(height - 5))
    end_point = (end_point[0],end_point[1])
    img = cv2.arrowedLine(res0, start_point, end_point,(0,0,255), 3, tipLength = 0.5)
    #printImg(img,"final result")
    #print("--- %s seconds ---" % (time.time() - start_time))
    return img, LR_res


# In[10]:


def init_cam():
    frameWidth = 640
    frameHeight = 480
    cap = cv2.VideoCapture(0)
    cap.set(3,frameWidth)
    cap.set(4,frameHeight)
    return cap


# In[11]:
stop_thread = False
COR_res = 0

def capture_res(cap,show_res = True): # if needed, make it to true
    success,img = cap.read()
    img, LR_res = rdDetect(img)
    if 0:
        if (LR_res == 0):
            print("center")
        elif LR_res == 1:
            print("deviation to right")
        elif LR_res == 2:
            print("deviation to left")
    if show_res:
        cv2.imshow('result',img)
    return LR_res
    
def COR_detection():
    global COR_res
    cap = init_cam()
    while True:
        LR_res = capture_res(cap,True)
        COR_res = LR_res
        if cv2.waitKey(1)&0xFF == ord('q'):
            break
        if (stop_thread):
            cap.release()
            break

def COR_corr_func():
    global COR_res
    time.sleep(2)
    print("center of road correcting.................")
    if COR_res == 2:
        left()
        time.sleep(0.2)
        stop()
        time.sleep(2)
    if COR_res == 1:
        right()
        time.sleep(0.2)
        stop()
        time.sleep(2)
            




## ----------- main process ------------ ##

## PWM SETUP
GPIO.setmode(GPIO.BCM)
GPIO.setup(0, GPIO.OUT)
GPIO.setup(1, GPIO.OUT)
pwmR = GPIO.PWM(0, 100)
pwmL = GPIO.PWM(1, 100)
dutyR = 65
dutyL = 65
pwmR.start(dutyR)
pwmL.start(dutyL) 
pwmR.ChangeDutyCycle(dutyR)
pwmL.ChangeDutyCycle(dutyL)
    

def main_process(des):
    global danger
    global stop_thread
    global heading
    global dutyR
    global dutyL
    correct_heading = -999
    
    gpio_init()
    GPIO.setmode(GPIO.BCM)
    
    # pwm setup
    
    global COR_res

    #nav = navi.navigation_sys(True, 0, des)
    cap = ped_det.init_cam()
    
    mode = 99
    update_heading = True
    counter = 5
    keep_turning = False
    count_err = 0
    COR_correction = False
    while True:
        #time.sleep(0.1)
        print("center of road result:", COR_res)
        
        if stop_thread:
            break
        
        # low error possiblity
        if COR_correction and COR_res != 0:
            COR_corr_func()
            continue
        if COR_res == 0:
            COR_correction = False
        
        if (counter == 0 and update_heading):
            correct_heading = heading
            update_heading = False
            print("calibrating......")
            print("new heading: ", correct_heading)
            stop()
            time.sleep(2)
        
        #print("lidar status: ", danger)
        print("correct heading:", correct_heading)
        print("current heading:", heading)
        if (1 and counter == 0):
            if (not keep_turning):
                #mode = nav.update_step()
                mode = 1
                time.sleep(0.5)
            #mode = 1
            if (mode == 2 or mode == 3):
                # update_heading = True
                # steering
                
                if (mode==3):
                    print("turning right")
                    if (not keep_turning):
                        stop()
                        time.sleep(5)
                        correct_heading = (correct_heading + 100) % 360
                        print("NEW TARGET HEADING: ", correct_heading)
                        keep_turning = True
                    
                    err_heading = (correct_heading - heading + 540) % 360 - 180
                    if (err_heading < -10):
                        keep_turning = False
                        # start center of road detection
                        COR_correction = True
                    #elif (err_heading < -10):
                        #mode = 2
                    else:
                        right()
                        time.sleep(0.3)
                        stop()
                        time.sleep(2)
                ## mode == 2
                else:
                    print("turning left")
                    if (not keep_turning):
                        stop()
                        time.sleep(3)
                        correct_heading = (heading - 90) % 360
                        print("NEW TARGET HEADING: ", correct_heading)
                        keep_turning = True
                    
                    err_heading = (correct_heading - heading + 540) % 360 - 180
                    if (abs(err_heading) < 10):
                        keep_turning = False
                        # start center of road detection
                        COR_correction = True
                    #elif (err_heading > 10):
                        #mode = 3
                    else:
                        left()
                        time.sleep(0.3)
                        stop()
                        time.sleep(2)
                    
            # go straight
            elif (mode == 1):
                #detect_res = danger # or ped_det.capture_res(cap, True)
                detect_res = danger or ped_det.capture_res(cap)
                if cv2.waitKey(1)&0xFF == ord('q'):
                    break
                if (detect_res):
                    stop()
                    print("obstacle or pedestrian ahead")
                    time.sleep(2)
                    continue
                
                if (count_err == 0):
                    err = (correct_heading - heading + 540) % 360 - 180
                    # forward
                    if (err > 5):
                        #dutyR = dutyR + 0
                        #stop()
                        #right()
                        #time.sleep(0.02)
                        dutyL = dutyL + 5
                        if (dutyL > 100):
                            dutyL = 100
                        #pwmR.ChangeDutyCycle(dutyR)
                        pwmL.ChangeDutyCycle(dutyL)
                    if (err < -5):
                        #dutyR = dutyR + 0
                        #stop()
                        #left()
                        #time.sleep(0.02)
                        dutyL = dutyL - 4
                        if (dutyL < 30):
                            dutyL = 20
                        #pwmR.ChangeDutyCycle(dutyR)
                        pwmL.ChangeDutyCycle(dutyL)
                    print("heading_error = ", err)
                    print("PWM_R = ", dutyR, "PWM_L = ", dutyL)

                count_err = (count_err + 1) % 1
                #detect_res = danger
                
                #print(detect_res)
                
                forward()
                print("forward")
                time.sleep(0.05)
            else:
                stop()
                print("stop")
                
            print("------------DIVIDER------------\n")
        
        else:
            print("System loading")
            time.sleep(1)
            counter = counter - 1
            

    
# main
if __name__ == "__main__":
    
    des = raw_input("Where do you want me to deliver?")
    p1 = threading.Thread(target=lidar_check)
    p2 = threading.Thread(target=main_process, args=(des,))
    p3 = threading.Thread(target=read_heading_thread)
    #p4 = threading.Thread(target=COR_detection)
    p1.start()
    p2.start()
    p3.start()
    #p4.start()

    try:
        while True:
            time.sleep(0.1)
            #print("waiting")
            
    except KeyboardInterrupt:
        stop_thread = True
        p1.join()
        p2.join()
        p3.join()
        #p4.join()
        
        print("system stopped")
    finally:
        lidar.stop()
        lidar.disconnect()
        stop()
        GPIO.cleanup()
    
    
        
        
        
