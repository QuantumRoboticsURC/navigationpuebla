#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D

class PygameSim():
    def __init__(self):
        rospy.init_node("Pygame",anonymous=True)

        rospy.Subscriber("/odometry",Pose2D,self.callback)

    def callback(self,data):
        self.x=data.x
        self.y=data.y
        self.theta=data.theta

    def main(self):
        print("hola1")
        while not rospy.is_shutdown():
            print("hola2")
            print(self.x," ",self.y)