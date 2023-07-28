#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
import time
import consts as const
x= 0
y=0

def draw(mask,color):
    global x
    contornos,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
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

rospy.init_node("center_and_aproach",anonymous=True)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
twist = Twist()
        
cap = cv2.VideoCapture(0)

blueLow = np.array([95,100,20], np.uint8)
blueHigh = np.array([125,255,255], np.uint8)

greenLow = np.array([45,100,20], np.uint8)
greenHigh = np.array([65,255,255], np.uint8)

redLow1 = np.array([0,100,20], np.uint8)
redHigh1 = np.array([5,255,255], np.uint8)

redLow2 = np.array([170,100,20], np.uint8)
redHigh2 = np.array([179,255,255], np.uint8)

cont = True
ret,frame = cap.read()
midpoint = 0
if(ret): 
    midpoint = frame.shape[1]/2

while not rospy.is_shutdown():
    ret,frame = cap.read()

    if ret == True:
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
        print(cont)
        if (b == True):
            if (cont == True):
                print("Piedra verde detectada")
                cont = False
        elif (a == True):
            if (cont == True):
                print("Piedra azul detectada")
                cont = False
        elif (c == True):
            if (cont == True):
                print("Piedra roja detectada")
                cont = False
        else:
            cont = True
            
        
        if (midpoint*2-x+const.ANGLE_ERROR >midpoint and midpoint*2-x-const.ANGLE_ERROR<midpoint):
            print("Esta en frente")
            twist.linear.x=.33
            twist.angular.z=0
        elif (midpoint>(2*midpoint-x) -const.ANGLE_ERROR):
            print("Esta a la izquierda")
            twist.linear.x=0.16
            twist.angular.z=-0.16
        elif ((2*midpoint-x) +const.ANGLE_ERROR >midpoint):
            print("Esta a la derecha")
            twist.linear.x=0.16
            twist.angular.z=0.16
        if(a == False and b==False and c ==False):
            twist.linear.x=0
            twist.angular.z=0

        

        cv2.imshow('video',frameFlip)


        if cv2.waitKey(1) & 0xFF ==ord('s'):
            break
        cmd_vel_pub.publish(twist)
        time.sleep(.1)
cap.release()
cv2.destroyAllWindows()

