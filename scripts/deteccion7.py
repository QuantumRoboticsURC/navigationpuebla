#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
import time
import consts as const
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool,Int32,Float64

class Center():
    def __init__(self):
        #ROS
        rospy.init_node("center_and_aproach",anonymous=True)
        rospy.Subscriber("/arm_movement",Bool,self.callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.image_pub = rospy.Publisher("/detection_image_raw",Image,queue_size=10)
        self.image_pub_arm = rospy.Publisher("/detection_arm_image_raw",Image,queue_size=10)
        self.deteccion= rospy.Publisher("/deteccion_roca",Bool,queue_size=1)
        self.joint1 = rospy.Publisher("arm_teleop/joint1",Float64,queue_size=1)
        self.joint2 = rospy.Publisher("arm_teleop/joint2_lab",Float64,queue_size=1)
        self.joint3 = rospy.Publisher("arm_lab/joint3",Int32,queue_size=1)
        self.gripper = rospy.Publisher("arm_teleop/prism",Float64,queue_size=1)
        self.move_arm = rospy.Publisher("/arm_movement",Bool,queue_size=1)
        #Position Variables
        self.x = 0
        self.y = 0
        self.midpoint = 0
        self.midheight = 0
        self.contador=0
        self.arm = False
        #Camera Variables
        print("cam2")
        self.cam_2 = cv2.VideoCapture("/dev/CAMERA_ARM")
        #Colors
        self.blueLow = np.array([95.0,115.35,63.9], np.uint8)
        self.blueHigh = np.array([113,255.0,255.0], np.uint8)
        self.greenLow = np.array([45.6,63.9,29.6], np.uint8)
        self.greenHigh = np.array([75.0,255,255], np.uint8)
        self.redLow1 = np.array([0.0,117.8,10.0], np.uint8)
        self.redHigh1 = np.array([4.6,255,255], np.uint8)
        self.redLow2 = np.array([170,117.8,10], np.uint8)
        self.redHigh2 = np.array([179,255,255], np.uint8)
        #Other variables
        self.rock=""
        self.rocks = []
        self.bridge = CvBridge()
        self.rate = rospy.Rate(200)
        self.inicio=0
        self.final=0
        self.tiempo=0

    def callback(self,data):
        # Bandera booleana 
        self.arm = data.data

    def get_center(self):
        #Se calcula el centro de la imagen
        ret1,frame1 = self.cam_2.read()
        if(ret1):
            self.midpoint = frame1.shape[1]/2
            self.midheight = frame1.shape[0]/2
        self.pos=2
        self.veces=0

    def draw(self,mask,color,frame):
        #En esta función se trabaja con Open CV para la detección del color de las rocas
        contornos, _ = cv2.findContours(mask,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contornos:
            #El area se calcula con open CV del objeto que se detecto.
            area = cv2.contourArea(c)
            #Limitamos el tamaño del area para que solo haga el proceso en el rango esperado.
            if area <7000 and area >1000:
                moments = cv2.moments(c)
                if(moments["m00"]==0): moments["m00"]=1
                self.x = int(moments["m10"]/moments["m00"])
                self.y = int(moments["m01"]/moments["m00"])
                nuevoContorno = cv2.convexHull(c)
                #Se dibuja en la imagen el perimero del objeto encontrado.
                cv2.circle(frame,(self.x,self.y), 5,(255,0,0),-1)
                cv2.drawContours(frame, [nuevoContorno],0,color,3)
                return True
        return False
    
    def cambiovel(self):
        #Se calcula la velocidad de giro dependiendo de la distancia a la que se encuentra el centro del objeto del centro de la camara
        if self.pos==-1 or self.pos==1:
            regla3=(abs(self.x - self.midpoint)-const.ANGLE_ERROR ) * 0.08 / (self.midpoint - const.ANGLE_ERROR ) + 0.08
            print(regla3)
            self.twist.linear.x=0
            self.twist.angular.z=regla3*self.pos
    
    
    def main(self):
        self.get_center()
        center_rock = False
        center_rock2= False
        cont = True
        count = 0
        print("ENTER MAIN")

        while not rospy.is_shutdown():
            self.ret1,self.frame1 = self.cam_2.read()
            if self.ret1:
     
                #Shows the image, this should only be run on the personal PC. Running it on the jetson will cause problems
                #cv2.imshow('video',frameFlip)
                #cv2.imshow('video1',frameFlip)
                #The image is shown in rqt_image_view
                if(not center_rock2):
                    self.move_arm.publish(False)
                    frameHSV = cv2.cvtColor(self.frame1,cv2.COLOR_BGR2HSV)
                    maskBlue = cv2.inRange(frameHSV,self.blueLow,self.blueHigh)
                    maskGreen = cv2.inRange(frameHSV,self.greenLow,self.greenHigh)
                    maskRed1= cv2.inRange(frameHSV,self.redLow1, self.redHigh1)
                    maskReed2 = cv2.inRange(frameHSV,self.redLow2,self.redHigh2)
                    maskRed = cv2.add(maskRed1,maskReed2)
                    #Checks if a rock has been detected 
                    a =self.draw(maskBlue,(255,0,0),self.frame1)
                    #b = self.draw(maskGreen,(0,255,0),self.frame1)
                    b=False
                    #Green was commented because in Puebla the floor is garden.
                    c = self.draw(maskRed,(0,0,255),self.frame1)
                    frameFlip = cv2.flip(self.frame1,1)
                    #Creates an image message with the contours drawn by the draw function
                    img_msg = self.bridge.cv2_to_imgmsg(self.frame1,"bgr8")
                    #Checks if the rock has been continuously detected
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
                    #Publish the flag to say that a rock was detected or not
                    self.deteccion.publish(detected)
                    #Checks if a rock was detected, it has to move to center the rock vertically
                    if (self.y+const.ANGLE_ERROR >self.midheight and self.y-const.ANGLE_ERROR<self.midheight and detected):
                        print("Esta en frente")
                        self.twist.linear.x=0
                        self.twist.angular.z=0
                        print("Eureka")
                        self.cmd_vel_pub.publish(self.twist)
                        center_rock=True   
                    elif (self.midheight - const.ANGLE_ERROR>self.y and detected):
                        print("Esta arriba")
                        self.twist.linear.x=0.08
                        self.twist.angular.z=0
                        self.cmd_vel_pub.publish(self.twist)
                        center_rock=False
                    elif (self.y >self.midheight + const.ANGLE_ERROR and detected):
                        print("Esta abajo")
                        self.twist.linear.x=-0.08
                        self.twist.angular.z=0
                        center_rock=False
                        self.cmd_vel_pub.publish(self.twist)

                    #If it was centered vertically it does the movement to center it horizontally
                    if center_rock:
                        if (self.midpoint*2-self.x+const.ANGLE_ERROR >self.midpoint and self.midpoint*2-self.x-const.ANGLE_ERROR<self.midpoint and detected):
                            self.pos=1
                            self.twist.linear.x=0
                            self.twist.angular.z=0
                            self.cmd_vel_pub.publish(self.twist)
                            print("Esta en frente")
                            center_rock2=True
                        elif (self.midpoint>(2*self.midpoint-self.x) -const.ANGLE_ERROR and detected):
                            if self.pos != 1:
                                inicio=time.time()
                                self.veces=0
                            else:
                                self.veces=time.time()-inicio
                            self.pos=1
                            print("Esta a la izquierda")
                            self.cambiovel()
                            self.cmd_vel_pub.publish(self.twist)
                            '''
                            twist.linear.x=0.0
                            twist.angular.z=-0.16
                            '''
                        elif ((2*self.midpoint-self.x) +const.ANGLE_ERROR >self.midpoint and detected):
                            if self.pos != -1:
                                inicio=time.time()
                                self.veces=0
                            else:
                                self.veces=time.time()-inicio
                            self.pos=-1
                            print("Esta a la derecha")
                            self.cambiovel()
                            self.cmd_vel_pub.publish(self.twist)
                            '''
                            twist.linear.x=0.0
                            twist.angular.z=-0.16
                            '''
                    
                if center_rock2:
                    #When the rock is centered it stopes moving and sends the flag to start the movement of the arm 
                    self.move_arm.publish(True)
                    self.deteccion.publish(True)
                    self.twist.linear.x=0
                    self.twist.angular.z=0
                    self.cmd_vel_pub.publish(self.twist)
                    count+=1
                    #print(count)
                    #It counts the time it lasts from the moment the movement begin until the moment the arm movement stops
                    #It helps so that it stops detecting the rock and the rover stops moving the wheels
                    if count==1:
                        self.inicio=rospy.get_time()
                    self.final=rospy.get_time()
                    self.tiempo=self.final-self.inicio
                    print(self.tiempo)
                    if self.tiempo>=66:
                        center_rock=False
                        center_rock2=False
                        count=0
                        self.move_arm.publish(False)
                        self.deteccion.publish(False)
                    #once the arm rutine ends, it starts to look for rocks again
                    
                
               
                if(not detected):
                    self.pos=2
                #If the rock has already been centered with the first camera, a second centering process starts with the arm camera

                #Publishes the image 
                img_msg = self.bridge.cv2_to_imgmsg(self.frame1,"bgr8")
                self.image_pub.publish(img_msg)
                img_msg_arm = self.bridge.cv2_to_imgmsg(self.frame1,"bgr8")
                self.image_pub_arm.publish(img_msg_arm)
                #self.image_pub_compressed.publish(img_msg_compressed.data)
                
            self.rate.sleep()
            #rospy.Rate(10).sleep()  
        cv2.destroyAllWindows()        
if __name__=="__main__":
    #Creates the object
    center = Center()
    #Starts doing the fuction main,until rospy is shutdown.
    while not rospy.is_shutdown():
        center.main()