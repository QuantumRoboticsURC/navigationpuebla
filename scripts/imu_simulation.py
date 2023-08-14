#!/usr/bin/env python3
import numpy as np
import rospy
import time
import consts as const
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation


class Imu_Sim():
    def __init__(self):
        rospy.init_node("ImuSim",anonymous=True)
        self.odom = Pose2D()
        self.imu_msg = Imu()
        self.listener_odom = rospy.Subscriber("/odometry",Pose2D,self.callback)
        self.pub_imu = rospy.Publisher("/imu/data",Imu,queue_size=10)
        self.rate = rospy.Rate(30)
        self.x = None
        self.y = None
        self.theta= None
        self.diff_x = None
        self.diff_y = None
        self.r = None
        self.pitch=None
        self.roll=None
        self.yaw = 0.0
        
        self.qx = None
        self.qy = None
        self.qz = None
        self.qw = None


    def callback(self,data):
        self.x=data.x
        self.y=data.y
        self.theta=data.theta

    def calculate(self,x,y):
        self.diff_x = (-2.0/5.0)*(x-4)
        self.diff_y = (-2.0/5.0)*(y-4)

        self.pitch= np.arctan(self.diff_x)
        self.roll= np.arctan(self.diff_y)
        self.yaw = 0.0

        self.r = Rotation.from_euler('yxz',[self.roll,self.pitch,self.yaw],degrees = False)

        quaternions = self.r.as_quat()

        #self.qx = np.sin(self.roll/2.0) * np.cos(self.pitch/2.0) * np.cos(self.yaw/2.0) - np.cos(self.roll/2.0) * np.sin(self.pitch/2.0) * np.sin(self.yaw/2.0)
        #self.qy = np.cos(self.roll/2.0) * np.sin(self.pitch/2.0) * np.cos(self.yaw/2.0) + np.sin(self.roll/2.0) * np.cos(self.pitch/2.0) * np.sin(self.yaw/2.0)
        #self.qz = np.cos(self.roll/2.0) * np.cos(self.pitch/2.0) * np.sin(self.yaw/2.0) - np.sin(self.roll/2.0) * np.sin(self.pitch/2.0) * np.cos(self.yaw/2.0)
        #self.qw = np.cos(self.roll/2.0) * np.cos(self.pitch/2.0) * np.cos(self.yaw/2.0) + np.sin(self.roll/2.0) * np.sin(self.pitch/2.0) * np.sin(self.yaw/2.0)

        #self.imu_msg.orientation.x = self.qx
        #self.imu_msg.orientation.y = self.qy
        #self.imu_msg.orientation.z = self.qz
        #self.imu_msg.orientation.w = self.qw

        self.imu_msg.orientation.x = quaternions[0]+0.02
        self.imu_msg.orientation.y = quaternions[1]+0.02
        self.imu_msg.orientation.z = quaternions[2]+0.02
        self.imu_msg.orientation.w = quaternions[3]+0.02

        self.pub_imu.publish(self.imu_msg)

    def main(self):
         
         while not rospy.is_shutdown():
            if (self.x is not None and self.y is not None):
                self.calculate(self.x,self.y)
            self.rate.sleep()

    
if __name__=="__main__":
    imu = Imu_Sim()
    imu.main()
