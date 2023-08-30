#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D

class PygameSim():
    def __init__(self):
        rospy.init_node("Pygame",anonymous=True)
        rospy.Subscriber("/odometry",Pose2D,self.callback)
        rospy.Subscriber("/cmd_vel",Twist,self.callback)
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        #Cmd_vel aún no implmentado
        self.vx=0.0
        self.vy=0.0
        self.vTheta=0.0

    def callback(self,data):
        self.x=data.x
        self.y=data.y
        self.theta=data.theta

    def main(self): #este es el main
        while not rospy.is_shutdown():
            x = float(self.x) #pose en x
            y = float(self.y) #pose en y
            theta = float(self.theta) #angulo

if __name__=="__main__":
    pygame = PygameSim()
    pygame.main()