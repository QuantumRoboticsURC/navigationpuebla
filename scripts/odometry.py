#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
import time
import consts as const

class Odometry():
    def __init__(self):
        rospy.init_node("Odometry",anonymous=True)
        self.vx=0.0
        self.vy=0.0
        self.vTheta=0.0
        self.angle = 0.0
        self.x = 0.0
        self.y = 0.0
        self.listener = rospy.Subscriber("/cmd_vel",Twist,self.callback)
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
        print("Current position:", self.x,",",self.y," at an angle of: ",self.angle)
  

    def main(self):
        while not rospy.is_shutdown():
            self.movement()
            self.rate.sleep()

if __name__=="__main__":
    odom = Odometry()
    odom.main()

    


        