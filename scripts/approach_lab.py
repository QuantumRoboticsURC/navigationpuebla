#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32,Float64

joint1 = rospy.Publisher("arm_teleop/joint1",Float64,queue_size=1)
joint2 = rospy.Publisher("arm_teleop/joint2_lab",Float64,queue_size=1)
joint3 = rospy.Publisher("arm_lab/joint3",Int32,queue_size=1)

while not rospy.is_shutdown():
    print("Home")
    joint1.publish(0)
    joint2.publish(130)
    joint3.publish(300)

    time.sleep(10)

    print("Intermediate")
    joint1.publish(0)
    joint2.publish(65)
    joint3.publish(358)

    time.sleep(10)

    print("Ground") #Cuidado que se mata
    joint1.publish(0)
    joint2.publish(-24)
    joint3.publish(430)

    time.sleep(10)

    print("Intermediate")
    joint1.publish(0)
    joint2.publish(65)
    joint3.publish(358)

    time.sleep(10)

    print("Ground Ex") #Probar en la lap de chevez con
    joint1.publish(0) #valores de ground normal
    joint2.publish(-24)
    joint3.publish(430)

    time.sleep(10)

    print("Intermediate")
    joint1.publish(0)
    joint2.publish(65)
    joint3.publish(358)

    time.sleep(10)