#!/usr/bin/env python3
import pyzed.sl as sl
import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
import time
import consts as const
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool

class Center():
    def __init__(self):
        #ROS
        rospy.init_node("center_and_aproach",anonymous=True)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.image_pub = rospy.Publisher("/detection_image_raw",Image,queue_size=10)
        self.image_pub_arm = rospy.Publisher("/detection_arm__image_raw",Image,queue_size=10)
        self.deteccion= rospy.Publisher("/deteccion_roca",Bool,queue_size=10)
        #self.image_pub_compressed = rospy.Publisher("/detection_image_compressed",CompressedImage,queue_size=10)
        #self.image_pub_arm_compressed = rospy.Publisher("/detection_arm__image_compressed",CompressedImage,queue_size=10)
        #Position Variables
        self.x = 0
        self.y = 0
        self.midpoint = 320
        self.midheight = 120
        #
        self.zed_camera = sl.Camera()
        self.zed_init_params = sl.InitParameters()
        self.zed_init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use PERFORMANCE depth mode
        self.zed_init_params.camera_resolution = sl.RESOLUTION.HD720
        err = self.zed_camera.open(self.zed_init_params)

        if err != sl.ERROR_CODE.SUCCESS:
            exit(1)
        rospy.sleep(1.0)
        print("awebo")

        self.zed_runtime_parameters = sl.RuntimeParameters()
        self.zed_runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL  # Use STANDARD sensing mode
        #sl.SENSING_MODE.FILL
        self.zed_runtime_parameters.confidence_threshold = 100
        self.zed_runtime_parameters.textureness_confidence_threshold = 100
        self.image_size = self.zed_camera.get_camera_information().camera_resolution
        self.image_size.width = 640
        self.image_size.height = 360

        self.image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
        self.depth_image_zed = sl.Mat(self.image_size.width, self.image_size.height, sl.MAT_TYPE.U8_C4)
        self.point_cloud = sl.Mat()
        print("cam2")
        self.cam_2 = cv2.VideoCapture("/dev/CAMERA_ARM")
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
        self.bridge = CvBridge()
        self.rate = rospy.Rate(10)

    def get_center(self):
        ret,frame = self.cam_1.read()
        ret1,frame1 = self.cam_2.read()
        if(ret): 
            self.midpoint = frame.shape[1]/2
        if(ret1):
            self.midheight = frame1.shape[2]/2
        self.pos=2
        self.veces=0

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
    
    def cambiovel(self):
        
        if self.pos==-1 or self.pos==1:
            if self.veces<=2:
                self.twist.linear.x=0
                self.twist.angular.z=self.veces*.08*self.pos
            else:
                regla3=(abs(self.x - self.midpoint)-const.ANGLE_ERROR ) * 0.08 / (self.midpoint - const.ANGLE_ERROR ) + 0.08
                print(regla3)
                self.twist.linear.x=0
                self.twist.angular.z=regla3*self.pos
    
    
    def main(self):
        self.get_center()
        center_rock = False
        cont = True
        while not rospy.is_shutdown():
            self.zed_camera.retrieve_image(self.image_zed, sl.VIEW.LEFT, sl.MEM.CPU, self.image_size)
            self.frame = self.image_ocv = self.image_zed.get_data()
            if True:
                #reads frames
                frameHSV = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)
                maskBlue = cv2.inRange(frameHSV,self.blueLow,self.blueHigh)
                maskGreen = cv2.inRange(frameHSV,self.greenLow,self.greenHigh)
                maskRed1= cv2.inRange(frameHSV,self.redLow1, self.redHigh1)
                maskReed2 = cv2.inRange(frameHSV,self.redLow2,self.redHigh2)
                maskRed = cv2.add(maskRed1,maskReed2)
                #Checks if a rock has been detected 
                a =self.draw(maskBlue,(255,0,0),self.frame)
                b = self.draw(maskGreen,(0,255,0),self.frame)
                c = self.draw(maskRed,(0,0,255),self.frame)
                frameFlip = cv2.flip(self.frame,1)
                #Creates an image message with the contours drawn by the draw function
                img_msg = self.bridge.cv2_to_imgmsg(self.frame,"bgr8")
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

                self.deteccion.publish(detected)

                #Shows the image, this should only be run on the personal PC. Running it on the jetson will cause problems
                #cv2.imshow('video',frameFlip)

                #Code that Checks if the rock is on the right or on the left. Or if it has already been centered 

                if (self.midpoint*2-self.x+const.ANGLE_ERROR >self.midpoint and self.midpoint*2-self.x-const.ANGLE_ERROR<self.midpoint and detected):
                    self.pos=1
                    self.twist.linear.x=0
                    self.twist.angular.z=0
                    self.cmd_vel_pub.publish(self.twist)
                    print("Esta en frente")
                    self.twist.linear.x=.20
                    self.twist.angular.z=0
                    #time.sleep(1)
                    self.cmd_vel_pub.publish(self.twist)
                    #time.sleep(2)
                    center_rock = False #This should be True, currently for testing it is set to False. 
                elif (self.midpoint>(2*self.midpoint-self.x) -const.ANGLE_ERROR and detected):
                    if self.pos != 1:
                        inicio=time.time()
                        self.veces=0
                    else:
                        self.veces=time.time()-inicio
                    self.pos=1
                    print("Esta a la izquierda")
                    self.cambiovel()
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
                    '''
                    twist.linear.x=0.0
                    twist.angular.z=-0.16
                    '''
                    
               #This part of the code has to be reviewd once the search routine is determined
                if(not detected):
                    self.pos=2
                    self.twist.linear.x=0
                    self.twist.angular.z=0
                #If the rock has already been centered with the first camera, a second centering process starts with the arm camera
                if center_rock:
                    ret1,self.frame1 = self.cam_2.read()
                    if ret1:
                        frameFlip = cv2.flip(self.frame1,1)
                        a =self.draw(maskBlue,(255,0,0),self.frame1)
                        b = self.draw(maskGreen,(0,255,0),self.frame1)
                        c = self.draw(maskRed,(0,0,255),self.frame1) 
                        detected = a == True or b==True or c ==True

                        img_msg_arm = self.bridge.cv2_to_imgmsg(self.frame,"bgr8")
                        self.image_pub_arm.publish(img_msg_arm)
                        #cv2.imshow('video1',frameFlip)
                        if (self.y+const.DISTANCE_ERROR >self.midheight and self.y-const.DISTANCE_ERROR<self.midheight and detected):
                            print("Esta en frente")
                            self.twist.linear.x=0
                            self.twist.angular.z=0
                            print("Eureka")
                            cv2.destroyAllWindows()   
                            break
                        elif (self.midheight<self.y +const.DISTANCE_ERROR and detected):
                            print("Esta abajo")
                            self.twist.linear.x=0.08
                            self.twist.angular.z=0
                        elif (self.y-const.DISTANCE_ERROR <self.midheight and detected):
                            print("Esta arriba")
                            self.twist.linear.x=-0.08
                            self.twist.angular.z=0

                self.cmd_vel_pub.publish(self.twist)
                self.image_pub.publish(img_msg)
                #self.image_pub_compressed.publish(img_msg_compressed.data)
                
            self.rate.sleep()
            #rospy.Rate(10).sleep()  
        cv2.destroyAllWindows()        
if __name__=="__main__":
    center = Center()
    center.main()

