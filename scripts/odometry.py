#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from navigationpuebla.msg import odom
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
import time
import consts as const
import math

class Odometry():
    def __init__(self):
        #We start the node 
        rospy.init_node("Odometry",anonymous=True)
        #We declare the variables for the speed, position and angles
        self.vx=0.0
        self.vy=0.0
        self.vTheta=0.0
        self.angle = 0.0
        self.x = 0.0
        self.y =0.
        #We declare two variables for the messages, one is the geometry messages Pose2D and the custom message Odom()
        self.odom = odom()
        self.odometry = Pose2D()
        #We subscribe to cmd_vel to calculate velocities 

        rospy.Subscriber("/cmd_vel",Twist,self.callback)
        #We create publishers for both messages 
        self.pub_odom = rospy.Publisher("/odom",odom,queue_size=10)
        self.pub_odometry = rospy.Publisher("/odometry",Pose2D,queue_size=10)

        #We create the rate 
        self.rate = rospy.Rate(30)
        
 
    def callback(self,data):
        #We calculate the velocities based on the angle with the correction for the robot's real movement and orientation
        self.vx = data.linear.x*np.cos(self.angle/const.ODOM_ANGLE_CORRECTION)
        self.vy = data.linear.x*np.sin(self.angle/const.ODOM_ANGLE_CORRECTION)
        self.vTheta = data.angular.z

        #Using Euler's method we calculate the x,y,Theta coordinates from the robot based of their velocities. 
    def movement(self):
        
        dT = 1.0/30.0 #the time differential based on the refresh rate
        self.x += self.vx*(dT) 
        self.y += self.vy*(dT)
        self.angle += self.vTheta*(dT)

        #We make a mapping so that the angle is always between 0 and 2pi
        if(abs(self.angle) > (2*math.pi)):
            self.angle = self.angle%2*math.pi
        
        if(self.angle<0):
            self.angle = 0

        #We publish the messages with the correction of the real world
        self.odom.x= float(self.x/const.ODOM_DISTANCE_CORRECTION)
        self.odom.y = float(self.y/const.ODOM_DISTANCE_CORRECTION)
        self.odom.theta = float(self.angle/const.ODOM_ANGLE_CORRECTION)
        self.pub_odom.publish(self.odom)
        
        self.odometry.x = float(self.x/const.ODOM_DISTANCE_CORRECTION)
        self.odometry.y=float(self.y/const.ODOM_DISTANCE_CORRECTION)
        self.odometry.theta=float(self.angle/const.ODOM_ANGLE_CORRECTION)
        self.pub_odometry.publish(self.odometry)
        
        print("Vx",self.vx," Vy",self.vy," Vtheta",self.vTheta)
        print("Current position:", self.x/const.ODOM_DISTANCE_CORRECTION,",",self.y/const.ODOM_DISTANCE_CORRECTION," at an angle of: ",self.angle/const.ODOM_ANGLE_CORRECTION)

    def main(self):
        while not rospy.is_shutdown():
            self.movement()
            self.rate.sleep()

if __name__=="__main__":
    odom = Odometry()
    odom.main()

    


        
