#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Int32,Float64

rospy.init_node("aproach_lab",anonymous=True)

joint1 = rospy.Publisher("arm_teleop/joint1",Float64,queue_size=1)
joint2 = rospy.Publisher("arm_teleop/joint2_lab",Float64,queue_size=1)
joint3 = rospy.Publisher("arm_lab/joint3",Int32,queue_size=1)
gripper = rospy.Publisher("arm_teleop/prism",Float64,queue_size=1)

while not rospy.is_shutdown():
    time.sleep(.5)
    print("Home")
    joint3.publish(360)
    time.sleep(.5)
    joint1.publish(0)
    joint2.publish(130)
    

    time.sleep(3)

    print("Intermediate")
    joint3.publish(370)
    time.sleep(.5)
    joint1.publish(0)
    joint2.publish(65)
    

    time.sleep(5)

    gripper.publish(-1)
    time.sleep(4)
    gripper.publish(0)
    # print("Ground") #Cuidado que se mata
    # joint1.publish(0)
    # joint2.publish(-24)
    # joint3.publish(430)

    # time.sleep(10)

    # print("Intermediate")
    # joint1.publish(0)
    # joint2.publish(65)
    # joint3.publish(358)

    # time.sleep(10)

    print("Ground Ex") #Probar en la lap de chevez con
    joint3.publish(415)
    time.sleep(.5)
    joint1.publish(0) #valores de ground normal
    joint2.publish(5)
    

    time.sleep(10)

    gripper.publish(1)
    time.sleep(4)
    gripper.publish(0)

    print("Intermediate")
    joint1.publish(0)
    joint2.publish(65)
    time.sleep(2)
    joint3.publish(370)

    time.sleep(3)

    print("Home")
    joint3.publish(360)
    time.sleep(.5)
    joint1.publish(0)
    joint2.publish(130)

    time.sleep(5)

    print("Caja")
    joint3.publish(360)
    time.sleep(.5)
    joint1.publish(85)
    joint2.publish(130)

    time.sleep(10)
    print("caja2")
    joint1.publish(85)
    joint2.publish(160)
    time.sleep(2)
    joint3.publish(350)
    time.sleep(.5)

    gripper.publish(-1)
    time.sleep(5)
    gripper.publish(0)

    gripper.publish(1)
    time.sleep(7)
    gripper.publish(0)

    print("Home")
    joint3.publish(360)
    time.sleep(.5)
    joint1.publish(0)
    joint2.publish(130)

    time.sleep(10)




