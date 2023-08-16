#!/usr/bin/env python3

import rospy
import time as tm
from std_msgs.msg import Int32

servoRight = rospy.Publisher("servo_right",Int32,queue_size=1)
servoCenter = rospy.Publisher("servo_center",Int32,queue_size=1)
servoLeft = rospy.Publisher("servo_left",Int32,queue_size=1)