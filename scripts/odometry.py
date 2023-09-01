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
        rospy.init_node("Odometry",anonymous=True)
        self.vx=0.0
        self.vy=0.0
        self.vTheta=0.0
        self.angle = 0.0
        self.x = 0.0
        self.y =0.
        self.odom = odom()
        self.odometry = Pose2D()
        rospy.Subscriber("/cmd_vel",Twist,self.callback)
        self.pub_odom = rospy.Publisher("/odom",odom,queue_size=10)
        self.pub_odometry = rospy.Publisher("/odometry",Pose2D,queue_size=10)
        self.previous_time = rospy.get_time()
        self.rate = rospy.Rate(30)
        self.dir = 1
    
 
    def callback(self,data):
        self.vx = data.linear.x*np.cos(self.angle/const.ODOM_ANGLE_CORRECTION)
        self.vy = data.linear.x*np.sin(self.angle/const.ODOM_ANGLE_CORRECTION)
        self.vTheta = data.angular.z

        
    def movement(self):
        dT = 1.0/30.0
        self.x += self.vx*(dT)
        self.y += self.vy*(dT)
        self.angle += self.vTheta*(dT)

        if(abs(self.angle) > (2*math.pi)):
            self.angle = self.angle%2*math.pi
        
        if(self.angle<0):
            self.angle = 0

        self.odom.x= float(self.x)
        self.odom.y = float(self.y)

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

    


        
