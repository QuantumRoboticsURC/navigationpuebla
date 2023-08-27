#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D

class PygameSim():
    def_init_(self):
    rospy.init_node("Pygame",anonymous=True)

    rospy.Subscriber("/odometry",Pose2D,self.callback)

    self.x=0.0
    self.y=0.0
    self.theta=0.0
    self.coordinates = []
    self.posxa=self.x
    self.posya=self.y

    def main(self):
        while not rospy.is_shutdown():
            pritn(self.x," ",self.y)