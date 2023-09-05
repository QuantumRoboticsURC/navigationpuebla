#!/usr/bin/env python3.7

import roslaunch
import rospy

rospy.init_node('roslaunchpy', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/root/catkin_ws/src/navigationpuebla/launch/puebla.launch"])
launch.start()
rospy.loginfo("started")

#rospy.sleep(3)
# 3 seconds later
#launch.shutdown()