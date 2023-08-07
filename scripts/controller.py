#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Bool
from navigationpuebla.msg import odom,target
import time
import consts as const
import math

class Controller():
    def __init__(self):
        rospy.init_node("Controller")
        self.pub_target = rospy.Publisher("/target_coordninates",target,queue_size=10)
        self.listener = rospy.Subscriber("/go_to",Bool,self.callback)
        self.arrived = False
        self.coordinates = [(1,1),(7,1),(7,7),(7,1)]

    def callback(self,data):
        self.arrived=data
        
