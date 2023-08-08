#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Bool
from navigationpuebla.msg import odom,target
import time
import consts as const
import math
import route as route
class Controller():
    def __init__(self):
        rospy.init_node("Controller")
        self.pub_target = rospy.Publisher("/target_coordinates",target,queue_size=10)
        self.listener = rospy.Subscriber("/go_to",Bool,self.callback)
        self.arrived = False
        self.coordinates = [(1,1),(7,1),(7,7),(7,1)]
        self.target = target()
        self.rate = rospy.Rate(40)

    def callback(self,data):
        self.arrived=data

    def main(self):
        while not rospy.is_shutdown():
            for coordinate in self.coordinates:
                self.target.x=coordinate[0]
                self.target.y=coordinate[1]
                self.pub_target.publish(self.target)

if __name__ =="__main__":
    controller = Controller()
    controller.main()