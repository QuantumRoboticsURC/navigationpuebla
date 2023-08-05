#!/usr/bin/env python3

import rospy
from std_msgs.msg import String,Float64,Int32
import time

rospy.init_node("aproach",anonymous=True)

joint1=rospy.Publisher("arm_teleop/joint1",Float64,queue_size=1)
joint2=rospy.Publisher("arm_teleop/joint2",Float64,queue_size=1)
joint3=rospy.Publisher("arm_teleop/joint3",Float64,queue_size=1)
joint4=rospy.Publisher("arm_teleop/joint4",Float64,queue_size=1)
cam=rospy.Publisher("arm_teleop/cam",Int32,queue_size=1)
while not rospy.is_shutdown():
    joint1.publish(0)
    joint2.publish(160.19)
    joint3.publish(-163.3)
    joint4.publish(3.11)
    cam.publish(50)

    time.sleep(10)

    joint1.publish(0)
    joint2.publish(147.68)
    joint3.publish(-115.36)
    joint4.publish(-32.32)

    time.sleep(10)

    joint1.publish(0)
    joint2.publish(147.68)
    joint3.publish(-115.36)
    joint4.publish(-102.32)
    cam.publish(10)
    
    time.sleep(10)

    joint1.publish(0)
    joint2.publish(147.68)
    joint3.publish(-115.36)
    joint4.publish(-32.32)

    time.sleep(10)