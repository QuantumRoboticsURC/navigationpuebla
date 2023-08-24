#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

def nothing(x):
    pass

def draw(mask,color):
    _, contornos, _= cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contornos:
        area = cv2.contourArea(c)
        if area > 3000:
            moments = cv2.moments(c)
            if(moments["m00"]==0): moments["m00"]=1
            x = int(moments["m10"]/moments["m00"])
            y = int(moments["m01"]/moments["m00"])
            nuevoContorno = cv2.convexHull(c)
            cv2.circle(frame,(x,y), 5,(255,0,0),-1)
            cv2.drawContours(frame, [nuevoContorno],0,color,3)
            return True
    return False
    
        
cap = cv2.VideoCapture("/dev/CAMERA_ZED2I")
rospy.init_node("calibration")
cv2.namedWindow('Trackbars')
bridge = CvBridge()
image_pub = rospy.Publisher("/calibration_img",Image,queue_size=10)
cv2.createTrackbar('blueLow', 'Trackbars',0,100,nothing)
cv2.createTrackbar('BlueHigh', 'Trackbars',100,100,nothing)

cv2.createTrackbar('greenLow', 'Trackbars',0,100,nothing)
cv2.createTrackbar('greenHigh', 'Trackbars',100,100,nothing)

cv2.createTrackbar('redLow1', 'Trackbars',0,100,nothing)
cv2.createTrackbar('redHigh1', 'Trackbars',100,100,nothing)
cv2.createTrackbar('redLow2','Trackbars',0,100,nothing)
cv2.createTrackbar('redHigh2','Trackbars',100,100,nothing)

cont = True
while True:

    

    bl=cv2.getTrackbarPos('blueLow', 'Trackbars')
    bh=cv2.getTrackbarPos('blueHigh', 'Trackbars')

    gl=cv2.getTrackbarPos('greenLow', 'Trackbars')
    gh=cv2.getTrackbarPos('greenHigh', 'Trackbars')

    rl1=cv2.getTrackbarPos('redLow1', 'Trackbars')
    rh1=cv2.getTrackbarPos('redHigh1', 'Trackbars')

    rl2=cv2.getTrackbarPos('redLow2', 'Trackbars')
    rh2=cv2.getTrackbarPos('redHigh2', 'Trackbars')


    valbluelow = 95 + (bl*0.01)*15
    valbluehigh = 110 + (bh*0.01)*15

    valgreenlow = 45 + (gl*0.01)*15
    valgreenhigh = 60 + (gh*0.01)*15

    valredlow1 = 0 + (rl1*0.01)*10
    valredhigh1 = 10 + (rh1*0.01)*10

    valredlow2 = 170 + (rl2*0.01)*5
    valredhigh2 = 175 + (rh2*0.01)*4




    blueLow = np.array([valbluelow,100,20], np.uint8)
    blueHigh = np.array([valbluehigh,255,255], np.uint8)

    greenLow = np.array([valgreenlow,100,20], np.uint8)
    greenHigh = np.array([valgreenhigh,255,255], np.uint8)

    redLow1 = np.array([valredlow1,100,20], np.uint8)
    redHigh1 = np.array([valredhigh1,255,255], np.uint8)

    redLow2 = np.array([valredlow2,100,20], np.uint8)
    redHigh2 = np.array([valredhigh2,255,255], np.uint8)

    ret,frame = cap.read()

    if ret == True:
        key = cv2.waitKey(1)
        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        maskBlue = cv2.inRange(frameHSV,blueLow,blueHigh)
        maskGreen = cv2.inRange(frameHSV,greenLow,greenHigh)
        maskRed1= cv2.inRange(frameHSV,redLow1, redHigh1)
        maskReed2 = cv2.inRange(frameHSV,redLow2,redHigh2)
        maskRed = cv2.add(maskRed1,maskReed2)
        a =draw(maskBlue,(255,0,0))
        b = draw(maskGreen,(0,255,0))
        c = draw(maskRed,(0,0,255))
        frameFlip = cv2.flip(frame,1)
        #if (b == True):
            #if (cont == True):
                #print("Piedra verde detectada")
                #cont = False
        #elif (a == True):
            #if (cont == True):
                #print("Piedra azul detectada")
                #cont = False
        #elif (c == True):
            #if (cont == True):
                #print("Piedra roja detectada")
                #cont = False
        #else:
            #cont = True

        if key == ord('p'):
            print("Valor blue Low: ",valbluelow,",",100,",",20)
            print("Valor blue High: ",valbluehigh,",",255,",",255)
            print("Valor green Low: ",valgreenlow,",",100,",",20)
            print("Valor green High: ",valgreenhigh,",",255,",",255)
            print("Valor red1 Low: ",valredlow1,",",100,",",20)
            print("Valor red1 High: ",valredhigh1,",",255,",",255)
            print("Valor red2 Low: ",valredlow2,",",100,",",20)
            print("Valor red2 Low: ",valredhigh2,",",255,",",255)


            
        
        img_msg = bridge.cv2_to_imgmsg(frame,"bgr8")
        image_pub.publish(img_msg)
        cv2.imshow('video',frameFlip)


        if key & 0xFF ==ord('s'):
            break
cap.release()
cv2.destroyAllWindows()