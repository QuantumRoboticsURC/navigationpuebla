#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
import time
import consts as const

class Route():
    def __init__(self):
        rospy.init_node("Route",anonymous=True)
        self.twist = Twist()
        self.pub_cmd = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.start_time = rospy.get_time()
        self.rate= rospy.Rate(1)

    def routine(self):
        current_time = rospy.get_time()
        if((current_time-self.start_time)%8!=0):
            self.twist.linear.x=0.33
        else:
            self.twist.angular.z=0.5
        self.pub_cmd.publish(self.twist)
        self.twist.linear.x=0.0
        self.twist.angular.z=0.0
    
    def main(self):
        while not rospy.is_shutdown():
            self.routine()
            self.rate.sleep()
    
if __name__=="__main__":
    route = Route()
    route.main()


