#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from navigationpuebla.msg import odom
import time
import consts as const
import math
class Route():
    def __init__(self):
        rospy.init_node("Route",anonymous=True)
        self.twist = Twist()
        self.pub_cmd = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.listener_odom = rospy.Subscriber("/odom",odom,self.callback)
        self.start_time = rospy.get_time()
        self.rate= rospy.Rate(1)
        self.x=0
        self.y=0
        self.theta=0
    
    def callback(self,data):
        self.x=data.x
        self.y=data.y
        self.theta=data.theta

    def routine(self):
        current_time = rospy.get_time()
        if((current_time-self.start_time)%8!=0):
            self.twist.linear.x=0.33
        else:
            self.twist.angular.z=0.5
        self.pub_cmd.publish(self.twist)
        self.twist.linear.x=0.0
        self.twist.angular.z=0.0
    
    def go_to(self,x1,y1):
        cuadrante = 0
        distance = np.sqrt(pow(x1-self.x,2)+pow(self.y-y1,2))
        angle = np.arctan(abs(self.x-x1),abs(self.y-y1))
        if(x1>self.x):
            if(y1>self.y):
                cuadrante = 1
            elif (self.y>y1):
                cuadrante = 4
                angle = 2*math.pi-angle
        elif(self.x>x1):
            if(y1>self.y):
                cuadrante = 2
                angle = math.pi-angle
            elif(self.y>y1):
                cuadrante =3
                angle = math.pi+angle
        
                




    
    def main(self):
        while not rospy.is_shutdown():
            self.routine()
            self.rate.sleep()
    
if __name__=="__main__":
    route = Route()
    route.main()


