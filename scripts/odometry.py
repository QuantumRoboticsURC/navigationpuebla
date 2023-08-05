#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from navigationpuebla.msg import odom
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
        self.y = 0.0
        self.odom = odom()
        self.listener_cmd_vel = rospy.Subscriber("/cmd_vel",Twist,self.callback)
        self.pub_odom = rospy.Publisher("/odom",odom,queue_size=10)
        self.previous_time = rospy.get_time()
        self.rate = rospy.Rate(30)

    def callback(self,data):
        self.vx = data.linear.x*np.cos(self.angle)
        self.vy = data.linear.x*np.sin(self.angle)
        self.vTheta = data.angular.z

        
    def movement(self):
        dT = 1.0/30.0
        self.x += self.vx*(dT)
        self.y += self.vy*(dT)
        self.angle += self.vTheta*(dT)
        self.previous_time = rospy.get_time()

        if(abs(self.angle) > (2*math.pi)):
            self.angle = self.angle%2*math.pi
        
        if(self.angle<0):
            self.angle = 2*math.pi+self.angle

        print("Current position:", self.x,",",self.y," at an angle of: ",self.angle)
        self.odom.x= float(self.x)
        self.odom.y = float(self.y)
        self.odom.theta = float(self.angle)
        self.pub_odom.publish(self.odom)

    def main(self):
        while not rospy.is_shutdown():
            self.movement()
            self.rate.sleep()

if __name__=="__main__":
    odom = Odometry()
    odom.main()

    


        