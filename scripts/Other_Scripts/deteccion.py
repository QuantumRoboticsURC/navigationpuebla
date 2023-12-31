#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from std_msgs.msg import String,Float64,Int32
from geometry_msgs.msg import Twist
import time
import consts as const
x=0
y=0

def draw(mask,color):
    global x,y
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

def y_axis_center(rock):
    global rocks,colors,midheight
    cam = cv2.VideoCapture("/dev/video5")
    time.sleep(1)
    print("Entered second camera ")
    while not rospy.is_shutdown():
        ret,frame = cam.read()
        print(ret)
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
            detected = a == True or b==True or c ==True
           # cv2.imshow('video1',frameFlip)
            print(y)
            if (y+const.DISTANCE_ERROR >midheight and y-const.DISTANCE_ERROR<midheight and detected):
                print("Esta en frente")
                twist.linear.x=0
                twist.angular.z=0
                rocks.append(rock)
                print("Eureka")
                break
            elif (midheight<y +const.DISTANCE_ERROR and detected):
                print("Esta abajo")
                twist.linear.x=0.08
                twist.angular.z=0
            elif (y-const.DISTANCE_ERROR <midheight and detected):
                print("Esta arriba")
                twist.linear.x=-0.08
                twist.angular.z=0
            cmd_vel_pub.publish(twist)   
            time.sleep(.1)      
                   
def cambiovel(pos,veces):
    global twist,x,midpoint
    if pos==-1 or pos==1:
        if veces<=2:
            twist.linear.x=0
            twist.angular.z=veces*.08*pos
        else:
            print(abs(x-midpoint))
            regla3=(abs(x - midpoint)-const.ANGLE_ERROR ) * 0.1 / (midpoint - const.ANGLE_ERROR ) + 0.1
            print(regla3)
            twist.linear.x=0
            twist.angular.z=regla3*pos
    



rospy.init_node("center_and_aproach",anonymous=True)
cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
twist = Twist()


predifined=rospy.Publisher('/predefined',String,queue_size=1)


joint1=rospy.Publisher("arm_teleop/joint1",Float64,queue_size=1)
joint2=rospy.Publisher("arm_teleop/joint2",Float64,queue_size=1)
joint3=rospy.Publisher("arm_teleop/joint3",Float64,queue_size=1)
joint4=rospy.Publisher("arm_teleop/joint4",Float64,queue_size=1)
cam=rospy.Publisher("arm_teleop/cam",Int32,queue_size=1)

cap = cv2.VideoCapture("/dev/CAMERA_ARM_LAB")

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
midheight=0
rocks = []
rock = ""
colors = {}
control = True
if(ret): 
    midpoint = frame.shape[1]/2
    midheight = frame.shape[2]/2
posicion=2
joint1.publish(0)
joint2.publish(160.19)
joint3.publish(-163.3)
joint4.publish(3.11)
cam.publish(50)

time.sleep(10)
while not rospy.is_shutdown() and control:
    ret,frame = cap.read()
    if ret == True:

        frameHSV = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        maskBlue = cv2.inRange(frameHSV,blueLow,blueHigh)
        maskGreen = cv2.inRange(frameHSV,greenLow,greenHigh)
        maskRed1= cv2.inRange(frameHSV,redLow1, redHigh1)
        maskReed2 = cv2.inRange(frameHSV,redLow2,redHigh2)
        maskRed = cv2.add(maskRed1,maskReed2)
        a = False
        b = False
        #a =draw(maskBlue,(255,0,0))
        #b = draw(maskGreen,(0,255,0))
        c = draw(maskRed,(0,0,255))
        frameFlip = cv2.flip(frame,1)

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
        
       # cv2.imshow('video',frameFlip)

        if (midpoint*2-x+const.ANGLE_ERROR >midpoint and midpoint*2-x-const.ANGLE_ERROR<midpoint and detected):
            twist.linear.x=0
            twist.angular.z=0
            cmd_vel_pub.publish(twist)
            if(posicion != 0):
                contador = 1
                print(contador)
            else:           
                contador +=1
                print(contador)
            posicion=0
            if(contador >= 10):
                print("Esta en frente")
                twist.linear.x=.16
                twist.angular.z=0
                time.sleep(1)
                cmd_vel_pub.publish(twist)
                joint1.publish(0)
                joint2.publish(147.68)
                joint3.publish(-115.36)
                joint4.publish(-32.32)

                time.sleep(5)
                
                joint1.publish(0)
                joint2.publish(147.68)
                joint3.publish(-115.36)
                joint4.publish(-102.32)
                cam.publish(10)
                time.sleep(10)
                joint1.publish(0)
                joint2.publish(147.68)
                joint3.publish(-115.36)
                joint4.publish(-32.32)
                time.sleep(5)
                joint1.publish(0)
                joint2.publish(160.19)
                joint3.publish(-163.3)
                joint4.publish(3.11)
                cam.publish(50)
                cv2.destroyWindow("video")
                control = False
                #y_axis_center(rock)
        elif (midpoint>(2*midpoint-x) -const.ANGLE_ERROR and detected):
            if posicion != -1:
                inicio=time.time()
                segundos=0
            else:
                segundos=time.time()-inicio
            posicion=-1
            print("Esta a la izquierda")
            '''
            twist.linear.x=0.0
            twist.angular.z=-0.16
            '''
            cambiovel(posicion,segundos)
        elif ((2*midpoint-x) +const.ANGLE_ERROR >midpoint and detected):
            if posicion != 1:
                inicio=time.time()
                segundos=0
            else:
                segundos=time.time()-inicio
            posicion=1
            print("Esta a la derecha")
            posicion=1
            '''
            twist.linear.x=0
            twist.angular.z=0.16
            '''
            cambiovel(posicion,segundos)
        
        if(not detected):
            posicion=2
            twist.linear.x=0
            twist.angular.z=0

        if cv2.waitKey(1) & 0xFF ==ord('s'):
            break
        cmd_vel_pub.publish(twist)
        time.sleep(.1)
cap.release()
cv2.destroyAllWindows()

