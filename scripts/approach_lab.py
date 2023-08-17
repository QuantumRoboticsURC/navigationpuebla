#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32,Float64

joint1=rospy.Publisher("arm_teleop/joint1",Float64,queue_size=1)
servoRight = rospy.Publisher("servo_right",Int32,queue_size=1)
servoCenter = rospy.Publisher("servo_center",Int32,queue_size=1)
servoLeft = rospy.Publisher("servo_left",Int32,queue_size=1)

while not rospy.is_shutdown():
    print("Home")
    servoRight.publish(0)
    servoCenter.publish(130)
    servoLeft.publish(300)

    time.sleep(10)

    print("Intermediate")
    servoRight.publish(0)
    servoCenter.publish(65)
    servoLeft.publish(358)

    time.sleep(10)

    print("Ground") #Cuidado que se mata
    servoRight.publish(0)
    servoCenter.publish(-24)
    servoLeft.publish(430)

    time.sleep(10)

    print("Intermediate")
    servoRight.publish(0)
    servoCenter.publish(65)
    servoLeft.publish(358)

    time.sleep(10)

    print("Ground Ex") #Probar en la lap de chevez con
    servoRight.publish(0) #valores de ground normal
    servoCenter.publish(-24)
    servoLeft.publish(430)

    time.sleep(10)

    print("Intermediate")
    servoRight.publish(0)
    servoCenter.publish(65)
    servoLeft.publish(358)

    time.sleep(10)