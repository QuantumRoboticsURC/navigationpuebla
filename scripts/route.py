#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String,Bool
from navigationpuebla.msg import odom,target
import time
import consts as const
import math

class Route():
    def __init__(self):
        rospy.init_node("Route",anonymous=True)
        self.twist = Twist()
        self.pub_cmd = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.pub_go_to = rospy.Publisher("/go_to",Bool,queue_size=10)
        rospy.Subscriber("/odom",odom,self.callback)
        self.start_time = rospy.get_time()
        self.rate= rospy.Rate(10)
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        self.velocity=0.33
        self.angular_velocity = 0.15
        self.coordinates = []
        self.arrived = False
    
    def callback(self,data):
        self.x=data.x
        self.y=data.y
        self.theta=data.theta


    def routine(self,param):
        if(param=="line"):
            self.coordinates=[(3,0)]
        elif(param=="angle45"):
            self.coordinates=[(3,3)]
        elif(param=="angle90"):
            self.coordinates=[(0,3)]
        elif(param=="route"):
            self.coordinates = [(1,7),(2,7),(2,1),(3,1),(3,7),(4,7),(4,1),(5,1),(5,7),(6,7),(6,1),(7,1),(7,7)]
        else:
            print("Default")
            self.coordinates = [(1,1)]

    def go_to(self,x1,y1):
        cuadrante = 0
        distance = np.sqrt(pow(x1-self.x,2)+pow(self.y-y1,2))

        if(x1-self.x!=0):
            angle = np.arctan(abs(self.y-y1)/(abs(self.x-x1)))
        else:
            if(y1>self.y):
                angle = math.pi/2
            else:
                angle=math.pi*3/2


        if(x1-self.x<0):
            if(y1-self.y)>=0:
                cuadrante = 1
                angle = angle
            else:
                cuadrante = 4
                angle = 2*math.pi-angle
        elif(x1-self.x>0):
            if(y1-self.y>=0):
                cuadrante = 2
                angle = math.pi-angle
            else:
                cuadrante =3
                angle = math.pi+angle

        if(angle<0.05):
            angle=0

        print("Target angle: ",angle)
        if(self.theta>angle):
            print("-Moving from angle ",self.theta, " to ",angle)
            self.angular_velocity = -self.angular_velocity
            while(self.theta>angle*const.ODOM_ANGLE_CORRECTION):
                if(self.theta-const.ODOM_ANGLE_ERROR<angle):
                    self.twist.linear.x=0.0
                    self.twist.angular.z=0
                    self.pub_cmd.publish(self.twist)
                    break
                else:
                    self.twist.linear.x=0
                    self.twist.angular.z=self.angular_velocity
                    self.pub_cmd.publish(self.twist)
        else:
            print("+Moving from angle",self.theta, " to ",angle)
            #self.angular_velocity = -self.angular_velocity
            while(self.theta<angle*const.ODOM_ANGLE_CORRECTION): 
                if(self.theta+const.ODOM_ANGLE_ERROR>angle):
                    self.twist.linear.x=0
                    self.twist.angular.z=0
                    self.pub_cmd.publish(self.twist)
                    break
                else:
                    self.twist.linear.x=0
                    self.twist.angular.z=self.angular_velocity
                    self.pub_cmd.publish(self.twist)
        
        if(self.angular_velocity<0):
            self.angular_velocity=-self.angular_velocity
        
        target_time = const.ODOM_DISTANCE_CORRECTION*distance/self.velocity+rospy.get_time()

        print("Coordinates: ",self.x," ,",self.y)
        print("Target coordinates: ",x1," ,",y1)

        while(target_time>rospy.get_time()):
            self.twist.linear.x=self.velocity
            self.twist.angular.z=0
            self.pub_cmd.publish(self.twist)
            if((self.x >x1*const.ODOM_DISTANCE_CORRECTION-const.POSITION_ERROR and self.x<x1*const.ODOM_DISTANCE_CORRECTION+const.POSITION_ERROR) and (self.y>y1*const.ODOM_DISTANCE_CORRECTION-const.POSITION_ERROR and self.y<y1*const.ODOM_DISTANCE_CORRECTION+const.POSITION_ERROR)):
                break
        
        print("Arrived")
        self.arrived=True
        self.pub_go_to.publish(self.arrived)
        self.twist.linear.x=0
        self.twist.angular.z=0
        self.pub_cmd.publish(self.twist)

    def main(self):
        self.routine("angle90")
        print(self.coordinates)
        while not rospy.is_shutdown():
            for coordinates in self.coordinates:
                print("Going to coordinate: ",self.coordinates.index(coordinates))
                self.go_to(coordinates[0],coordinates[1])
            break
            self.rate.sleep()
    


if __name__=="__main__":
    route = Route()
    route.main()


