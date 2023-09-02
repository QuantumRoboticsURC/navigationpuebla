#!/usr/bin/env python3

import cv2
import numpy as np
#import rospy
#from geometry_msgs.msg import Twist
import time
#import consts as const
#from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import Image, CompressedImage
#from std_msgs.msg import Bool

class Calibration():
    def __init__(self):
        #Position Variables
        self.x = 0
        self.y = 0
        #Camera Variables
        print("cam1") 
        self.cam_1 = cv2.VideoCapture("/dev/CAMERA_ARM")
        #Colors
        self.valueColor = "blue"
        self.hlow = 0 
        self.hhigh = 0
        self.hlow2 = 0 
        self.hhigh2 = 0
        self.slow = 0
        self.shigh = 0
        self.vlow = 0
        self.vhigh = 0

        self.h_l = np.array([0,0,0], np.uint8)
        self.h_h = np.array([0,0,0], np.uint8)

        self.h_l2 = np.array([0,0,0], np.uint8)
        self.h_h2 = np.array([0,0,0], np.uint8)

        
        
        
        #Other variables
        #self.rock=""
        #self.rocks = []
        #self.bridge = CvBridge()
        #elf.rate = rospy.Rate(10)

    def color(self,color):
        if(color == "blue"):
            
            self.valueColor = "blue"

            hl=cv2.getTrackbarPos('hueLow', 'Trackbars')
            hh=cv2.getTrackbarPos('hueHigh', 'Trackbars')

            sl=cv2.getTrackbarPos('satLow', 'Trackbars')
            sh=cv2.getTrackbarPos('satHigh', 'Trackbars')

            vl=cv2.getTrackbarPos('valLow', 'Trackbars')
            vh=cv2.getTrackbarPos('valHigh', 'Trackbars')

            self.hlow = 95 + (hl*0.01)*30
            self.hhigh = 95 + (hh*0.01)*30
            self.slow = 10 + (sl*0.01)*245
            self.shigh = 10 + (sh*0.01)*245
            self.vlow = 10 + (vl*0.01)*245
            self.vhigh = 10 + (vh*0.01)*245

            self.h_l = np.array([self.hlow,self.slow,self.vlow], np.uint8)
            self.h_h = np.array([self.hhigh,self.shigh,self.vhigh], np.uint8)

        elif(color == "green"):
            
            self.valueColor = "green"

            hl=cv2.getTrackbarPos('hueLow', 'Trackbars')
            hh=cv2.getTrackbarPos('hueHigh', 'Trackbars')

            sl=cv2.getTrackbarPos('satLow', 'Trackbars')
            sh=cv2.getTrackbarPos('satHigh', 'Trackbars')

            vl=cv2.getTrackbarPos('valLow', 'Trackbars')
            vh=cv2.getTrackbarPos('valHigh', 'Trackbars')

            self.hlow = 40 + (hl*0.01)*35
            self.hhigh = 40 + (hh*0.01)*35
            self.slow = 10 + (sl*0.01)*245
            self.shigh = 10 + (sh*0.01)*245
            self.vlow = 10 + (vl*0.01)*245
            self.vhigh = 10 + (vh*0.01)*245

            self.h_l = np.array([self.hlow,self.slow,self.vlow], np.uint8)
            self.h_h = np.array([self.hhigh,self.shigh,self.vhigh], np.uint8)

        elif(color == "red"):
     
    

            self.valueColor = "red"

            hl=cv2.getTrackbarPos('hueLow', 'Trackbars')
            hh=cv2.getTrackbarPos('hueHigh', 'Trackbars')

            hl2=cv2.getTrackbarPos('hueLow2', 'Trackbars')
            hh2=cv2.getTrackbarPos('hueHigh2', 'Trackbars')

            sl=cv2.getTrackbarPos('satLow', 'Trackbars')
            sh=cv2.getTrackbarPos('satHigh', 'Trackbars')

            vl=cv2.getTrackbarPos('valLow', 'Trackbars')
            vh=cv2.getTrackbarPos('valHigh', 'Trackbars')

            self.hlow = 0 + (hl*0.01)*10
            self.hhigh = 0 + (hh*0.01)*10
            self.hlow2 = 170 + (hl2*0.01)*9
            self.hhigh2 = 170 + (hh2*0.01)*9
            self.slow = 10 + (sl*0.01)*245
            self.shigh = 10 + (sh*0.01)*245
            self.vlow = 10 + (vl*0.01)*245
            self.vhigh = 10 + (vh*0.01)*245

            self.h_l = np.array([self.hlow,self.slow,self.vlow], np.uint8)
            self.h_h = np.array([self.hhigh,self.shigh,self.vhigh], np.uint8)

            self.h_l2 = np.array([self.hlow2,self.slow,self.vlow], np.uint8)
            self.h_h2 = np.array([self.hhigh2,self.shigh,self.vhigh], np.uint8)

    def nothing(self,x):
        pass

    def draw(self,mask,color,frame):
        contornos, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
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

        def nothing(x):
                pass
        
        if(self.valueColor == "red"):
        
            cv2.namedWindow('Trackbars')
            cv2.createTrackbar('hueLow', 'Trackbars',0,100,nothing)
            cv2.createTrackbar('hueHigh', 'Trackbars',100,100,nothing)

            cv2.createTrackbar('hueLow2', 'Trackbars',0,100,nothing)
            cv2.createTrackbar('hueHigh2', 'Trackbars',100,100,nothing)

            cv2.createTrackbar('satLow', 'Trackbars',0,100,nothing)
            cv2.createTrackbar('satHigh', 'Trackbars',100,100,nothing)

            cv2.createTrackbar('valLow', 'Trackbars',0,100,nothing)
            cv2.createTrackbar('valHigh', 'Trackbars',100,100,nothing)

            
        else:
            cv2.namedWindow('Trackbars')
            cv2.createTrackbar('hueLow', 'Trackbars',0,100,nothing)
            cv2.createTrackbar('hueHigh', 'Trackbars',100,100,nothing)

            cv2.createTrackbar('satLow', 'Trackbars',0,100,nothing)
            cv2.createTrackbar('satHigh', 'Trackbars',100,100,nothing)

            cv2.createTrackbar('valLow', 'Trackbars',0,100,nothing)
            cv2.createTrackbar('valHigh', 'Trackbars',100,100,nothing)

      

        cont = True
        #while not rospy.is_shutdown():
        while True:
            self.color(self.valueColor)
            ret,self.frame = self.cam_1.read()
            if ret:
                if(self.valueColor == "red"):
                    frameHSV = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)
                    maskRed1= cv2.inRange(frameHSV,self.h_l, self.h_h)
                    maskReed2 = cv2.inRange(frameHSV,self.h_l2,self.h_h2)
                    mask = cv2.add(maskRed1,maskReed2)

                else: 
                    frameHSV = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)
                    mask = cv2.inRange(frameHSV,self.h_l,self.h_h)
                #Checks if a rock has been detected 
                a =self.draw(mask,(255,0,0),self.frame)
                frameFlip = cv2.flip(self.frame,1)
                #Creates an image message with the contours drawn by the draw function
                #img_msg = self.bridge.cv2_to_imgmsg(self.frame,"bgr8")
                #img_msg_compressed = CompressedImage()
                #img_msg_compressed.header ="Compressed"
                #img_msg_compressed=self.bridge.cv2_to_compressed_imgmsg(self.frame)
                #Checks if the rock has been continuously detected
                if (a == True):
                    if (cont == True and self.valueColor == "green"):
                        print("Piedra verde detectada")
                        cont = False
                        rock = "Green"
                if (a == True):
                    if (cont == True and self.valueColor == "blue"):
                        print("Piedra azul detectada")
                        cont = False
                        rock = "Blue"
                if (a == True):
                    if (cont == True and self.valueColor == "red"):
                        print("Piedra roja detectada")
                        cont = False
                        rock = "Red"
                else:
                    cont = True

                key = cv2.waitKey(1)

                if key ==ord('p'):
                    if(self.valueColor == "red"):
                    
                        print("low value: ",self.hlow, ", ",self.slow, ", ", self.vlow)
                        print("high value: ",self.hhigh, ", ",self.shigh, ", ", self.vhigh)

                        print("low value2: ",self.hlow2, ", ",self.slow, ", ", self.vlow)
                        print("high value2: ",self.hhigh2, ", ",self.shigh, ", ", self.vhigh)
                        
                    else:
                        print("low value: ",self.hlow, ", ",self.slow, ", ", self.vlow)
                        print("high value: ",self.hhigh, ", ",self.shigh, ", ", self.vhigh)

                FG=cv2.bitwise_and(self.frame, self.frame, mask=mask)
                bgMask=cv2.bitwise_not(mask)
                BG=cv2.cvtColor(bgMask,cv2.COLOR_GRAY2BGR)
                final=cv2.add(FG,BG)
                frameflip2 = cv2.flip(final,1)
                cv2.imshow('final',frameflip2)



                cv2.imshow('cam', frameFlip)
                if cv2.waitKey(1) & 0xFF == ord('s'):
                    break

        self.cam_1.release()
                
               

                 
            #self.rate.sleep()
            #rospy.Rate(10).sleep()  
        cv2.destroyAllWindows()        
if __name__=="__main__":
    calibration = Calibration()
    #color = "blue"
    calibration.main()


        
