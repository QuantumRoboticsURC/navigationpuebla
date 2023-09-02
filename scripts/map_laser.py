#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np
class Mapping():
    def __init__(self):
        rospy.init_node("Mapping",anonymous=True)
        rospy.Subscriber("/scan",LaserScan,self.callback)
        self.scan = LaserScan()
        rospy.Rate(30)
        self.map = []
        self.temp_map = []
    def callback(self,data):
        self.temp_map = data.ranges
        print("_____________--")
        print(data)
    
    def medidas(self):
        print(self.temp_map)


    def main(self):
        while not rospy.is_shutdown():
            self.medidas()

if __name__=="__main__":
    print("INIT")
    maps = Mapping()
    maps.main()

