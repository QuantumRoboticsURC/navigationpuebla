#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
import time
import consts as const


class Center():
    def __init__(self):
        #ROS
        rospy.init_node("center_and_aproach",anonymous=True)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        #Position Variables
        self.x = 0
        self.y = 0
        self.midpoint = 0
        self.midheight = 0
        #Camera Variables 
        self.cam_1 = cv2.VideoCapture("/dev/video0")
        self.cam_2 = cv2.VideoCapture("/dev/video1")
        #Colors
        self.blueLow = np.array([95,100,20], np.uint8)
        self.blueHigh = np.array([125,255,255], np.uint8)
        self.greenLow = np.array([45,100,20], np.uint8)
        self.greenHigh = np.array([65,255,255], np.uint8)
        self.redLow1 = np.array([0,100,20], np.uint8)
        self.redHigh1 = np.array([5,255,255], np.uint8)
        self.redLow2 = np.array([170,100,20], np.uint8)
        self.redHigh2 = np.array([179,255,255], np.uint8)
        #Other variables
        self.rock=""
        self.rocks = []
    
    def get_center(self):
        ret,frame = self.cam_1.read()
        ret1,frame1 = self.cam_2.read()
        if(ret): 
            self.midpoint = frame.shape[1]/2
        if(ret1):
            self.midheight = frame1.shape[2]/2
    def draw(self,mask,color,frame):
        contornos,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contornos:
            area = cv2.contourArea(c)
            if area > 3000:
                moments = cv2.moments(c)
                if(moments["m00"]==0): moments["m00"]=1
                self.x = int(moments["m10"]/moments["m00"])
                self.y = int(moments["m01"]/moments["m00"])
                nuevoContorno = cv2.convexHull(c)
                cv2.circle(frame,(self.x,self.y), 5,(255,0,0),-1)
                cv2.drawContours(frame, [nuevoContorno],0,color,3)
                return True
        return False
    def main(self):
        self.get_center()
        center_rock = False
        while not rospy.is_shutdown():
            ret,self.frame = self.cam_1.read()
            ret1,self.frame1 = self.cam_2.read()
            if ret:
                frameHSV = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)
                maskBlue = cv2.inRange(frameHSV,self.blueLow,self.blueHigh)
                maskGreen = cv2.inRange(frameHSV,self.greenLow,self.greenHigh)
                maskRed1= cv2.inRange(frameHSV,self.redLow1, self.redHigh1)
                maskReed2 = cv2.inRange(frameHSV,self.redLow2,self.redHigh2)
                maskRed = cv2.add(maskRed1,maskReed2)
                a =self.draw(maskBlue,(255,0,0),self.frame)
                b = self.draw(maskGreen,(0,255,0),self.frame)
                c = self.draw(maskRed,(0,0,255),self.frame)
                frameFlip = cv2.flip(self.frame,1)

                if (b == True):
                    if (cont == True):
                        print("Piedra verde detectada")
                        cont = False
                        rock = "Green"
                elif (a == True):
                    if (cont == True):
                        print("Piedra azul detectada")
                        cont = False
                        rock = "Blue"
                elif (c == True):
                    if (cont == True):
                        print("Piedra roja detectada")
                        cont = False
                        rock = "Red"
                else:
                    cont = True
                    
                detected = a == True or b==True or c ==True

                cv2.imshow('video',frameFlip)

                if (self.midpoint*2-self.x+const.ANGLE_ERROR >self.midpoint and self.midpoint*2-self.x-const.ANGLE_ERROR<self.midpoint and detected):
                    self.twist.linear.x=0
                    self.twist.angular.z=0
                    self.cmd_vel_pub.publish(self.twist)
                    print("Esta en frente")
                    self.twist.linear.x=.20
                    self.twist.angular.z=0
                    time.sleep(1)
                    self.cmd_vel_pub.publish(self.twist)
                    time.sleep(2)
                    center_rock = True
                elif (self.midpoint>(2*self.midpoint-self.x) -const.ANGLE_ERROR and detected):
                    print("Esta a la izquierda")
                    self.twist.linear.x=0.0
                    self.twist.angular.z=-0.16
                elif ((2*self.midpoint-self.x) +const.ANGLE_ERROR >self.midpoint and detected):
                    print("Esta a la derecha")
                    self.twist.linear.x=0
                    self.twist.angular.z=0.16
                
                if(not detected):
                    self.twist.linear.x=0
                    self.twist.angular.z=0
                if center_rock:
                    if ret1:
                        frameFlip = cv2.flip(self.frame1,1)
                        a =self.draw(maskBlue,(255,0,0),self.frame1)
                        b = self.draw(maskGreen,(0,255,0),self.frame1)
                        c = self.draw(maskRed,(0,0,255),self.frame1) 
                        detected = a == True or b==True or c ==True
                        cv2.imshow('video1',frameFlip)
                        if (self.y+const.DISTANCE_ERROR >self.midheight and self.y-const.DISTANCE_ERROR<self.midheight and detected):
                            print("Esta en frente")
                            self.twist.linear.x=0
                            self.twist.angular.z=0
                            print("Eureka")
                            break
                        elif (self.midheight<self.y +const.DISTANCE_ERROR and detected):
                            print("Esta abajo")
                            self.twist.linear.x=0.08
                            self.twist.angular.z=0
                        elif (y-const.DISTANCE_ERROR <self.midheight and detected):
                            print("Esta arriba")
                            self.twist.linear.x=-0.08
                            self.twist.angular.z=0

                if cv2.waitKey(1) & 0xFF ==ord('s'):
                    break
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(.1)
        


