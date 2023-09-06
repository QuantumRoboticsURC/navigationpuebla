#!/usr/bin/env python3.7

import roslaunch
import rospy

while not rospy.is_shutdown():
    rospy.init_node('roslaunchpy', anonymous=True)
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/root/catkin_ws/src/navigationpuebla/launch/puebla.launch"])
    launch.start()
    rospy.loginfo("started")
    while True:
        i = 0
    break

# 3 seconds later
#launch.shutdown()