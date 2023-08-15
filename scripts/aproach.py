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
gripper=rospy.Publisher("arm_teleop/prism",Float64,queue_size=1)
while not rospy.is_shutdown():
    print("Home")
    joint1.publish(0)
    joint2.publish(160.19)
    joint3.publish(-163.3)
    joint4.publish(3.11)
    cam.publish(50)

    time.sleep(10)

    print("Intermediate")
    joint1.publish(0)
    joint2.publish(147.68)
    joint3.publish(-115.36)
    joint4.publish(-32.32)

    time.sleep(5)

    print("Ver piso")
    joint1.publish(0)
    joint2.publish(147.68)
    joint3.publish(-115.36)
    joint4.publish(-102.32)
    cam.publish(10)
    
    time.sleep(10)

    print("Intermediate")
    joint1.publish(0)
    joint2.publish(147.68)
    joint3.publish(-115.36)
    joint4.publish(-32.32)

    time.sleep(6)

    print("Floor")
    joint1.publish(0)
    joint2.publish(49.13)
    joint3.publish(-101.74)
    joint4.publish(-37.38)

    time.sleep(10)

    print("Floor")
    joint1.publish(0)
    joint2.publish(49.13)
    joint3.publish(-101.74)
    joint4.publish(-37.38)

    time.sleep(8)

    gripper.publish(-1)
    
    time.sleep(2.5)

    gripper.publish(0)

    print("Recoger roca")
    joint1.publish(0)
    joint2.publish(33.91)
    joint3.publish(-76.24)
    joint4.publish(-27.68)
    cam.publish(70)
    
    time.sleep(5)

    gripper.publish(1)
    
    time.sleep(2.5)

    gripper.publish(0)

    print("Floor")
    joint1.publish(0)
    joint2.publish(49.13)
    joint3.publish(-101.74)
    joint4.publish(-37.38)

    time.sleep(5)
    
    print("Intermediate")
    joint1.publish(0)
    joint2.publish(147.68)
    joint3.publish(-115.36)
    joint4.publish(-32.32)

    time.sleep(10)
    print("Intermediate")
    joint1.publish(0)
    joint2.publish(147.68)
    joint3.publish(-115.36)
    joint4.publish(-32.32)

    time.sleep(10)

