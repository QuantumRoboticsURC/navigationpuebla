#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool,Float64,Int32
import time
class ArmCollection():
    def __init__(self):
        rospy.init_node("Arm_collection",anonymous=True)
        self.move_arm = rospy.Publisher("/arm_movement",Bool,queue_size=10)
        self.joint1 = rospy.Publisher("arm_teleop/joint1",Float64,queue_size=1)
        self.joint2 = rospy.Publisher("arm_teleop/joint2_lab",Float64,queue_size=1)
        self.joint3 = rospy.Publisher("arm_lab/joint3",Int32,queue_size=1)
        self.gripper = rospy.Publisher("arm_teleop/prism",Float64,queue_size=1)
        rospy.Subscriber("/arm_movement",Bool,self.callback)
        self.arm = False

    def callback(self,data):
        self.arm = data.data

    def main(self):
        self.gripper.publish(-1)
        time.sleep(4)
        self.gripper.publish(0)
        print("Ground Ex")
        self.joint3.publish(415)
        time.sleep(.5)
        self.joint1.publish(0)
        self.joint2.publish(5)
        time.sleep(10)
        self.gripper.publish(1)
        time.sleep(4)
        self.gripper.publish(0)

        print("Intermediate")
        self.joint1.publish(0)
        self.joint2.publish(65)
        time.sleep(2)
        self.joint3.publish(370)

        time.sleep(3)

        print("Home")
        self.joint3.publish(360)
        time.sleep(.5)
        self.joint1.publish(0)
        self.joint2.publish(130)

        time.sleep(5)

        print("Caja")
        self.joint3.publish(360)
        time.sleep(.5)
        self.joint1.publish(85)
        self.joint2.publish(130)

        time.sleep(10)
        print("caja2")
        self.joint1.publish(85)
        self.joint2.publish(160)
        time.sleep(2)
        self.joint3.publish(350)
        time.sleep(.5)

        self.gripper.publish(-1)
        time.sleep(5)
        self.gripper.publish(0)

        self.gripper.publish(1)
        time.sleep(7)
        self.gripper.publish(0)

        print("Home")
        self.joint3.publish(360)
        time.sleep(.5)
        self.joint1.publish(0)
        self.joint2.publish(130)

        time.sleep(10)

        time.sleep(.5)
        print("Home")
        self.joint3.publish(360)
        time.sleep(.5)
        self.joint1.publish(0)
        self.joint2.publish(130)
        time.sleep(3)
        self.get_center()
        print("Intermediate")
        self.joint3.publish(370)
        time.sleep(.5)
        self.joint1.publish(0)
        self.joint2.publish(65)
        self.move_arm.publish(False)

if __name__=="__main__":
    arm = ArmCollection()
    while( not rospy.is_shutdown()):
        if(arm.arm):
            arm.main()


