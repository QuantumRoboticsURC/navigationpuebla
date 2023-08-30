#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D

class PygameSim():
    def __init__(self):
        rospy.init_node("Pygame",anonymous=True)
        rospy.Subscriber("/odometry",Pose2D,self.callback)
        self.odometry = Pose2D()
        self.rate= rospy.Rate(10)
        self.x=0.0
        self.y=0.0
        self.theta=0.0

    def callback(self,data):
        self.x=data.x
        self.y=data.y
        self.theta=data.theta

    def main(self):
        print("hola1")
        while not rospy.is_shutdown():
            print("hola2")
            print(self.x," ",self.y)

if __name__=="__main__":
    pygame = PygameSim()
    pygame.main()