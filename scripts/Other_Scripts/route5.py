#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String,Bool
from navigationpuebla.msg import odom,target
from navigationpuebla.srv import arm_routine, arm_routineRequest,arm_routineResponse
import time
import consts as const
import math

class Route():
    def __init__(self):
        rospy.init_node("Route",anonymous=True)

        self.twist = Twist()
        self.pub_cmd = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.pub_go_to = rospy.Publisher("/go_to",Bool,queue_size=10)

        rospy.Subscriber("/odometry",Pose2D,self.callback)
        rospy.Subscriber("/deteccion_roca",Bool,self.callback2)

        self.roca_detected=False
        self.start_time = rospy.get_time()
        self.rate= rospy.Rate(30)
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        self.velocity=0.18
        self.angular_velocity = 0.15
        self.coordinates = []
        self.arrived = False
        self.simulation = False
        #self.ODOM_ANGLE = const.ODOM_ANGLE_CORRECTION
        self.ODOM_DISTANCE = const.ODOM_DISTANCE_CORRECTION
        self.posxa=self.x
        self.posya=self.y
        self.control = 0
        self.count = 0
    
    def callback(self,data):
        self.x=data.x
        self.y=data.y
        self.theta=data.theta
    
    def callback2(self,data):
        self.roca_detected=data.data

    def routine(self,param):
        if(param=="line"):
            self.coordinates=[(4,0)]
        elif(param=="angle45"):
            self.coordinates=[(3,3)]
        elif(param=="angle90"):
            self.coordinates=[(0,3)]
        elif(param=="zig"):
            self.coordinates=[(0,3),(3,0)]
        elif(param=="route"):
            self.coordinates = [(6,0),(6,1),(0.5,1),(0.5,3),(6,3),(6,5)]

            #self.coordinates = [(1,7),(2,7),(2,1),(3,1),(3,7),(4,7),(4,1),(5,1),(5,7),(6,7),(6,1),(7,1),(7,7)]
        else:
            print("Default")
            self.coordinates = [(1,1)]
    
    def move_angle(self,angle):
        if(self.theta>angle):
            print("-Moving from angle ",self.theta, " to ",angle)
            while(self.theta>angle and not self.roca_detected):
                if(self.theta<=angle):
                    self.twist.linear.x=0.0
                    self.twist.angular.z=0
                    self.pub_cmd.publish(self.twist)
                    break
                else:

                    self.twist.linear.x=0
                    self.twist.angular.z=-(abs((self.theta - 0)) * (self.angular_velocity- 0.08) / (2*math.pi - 0) + 0.08)
                    self.pub_cmd.publish(self.twist)
        else:
            print("+Moving from angle",self.theta, " to ",angle)
            while(self.theta<angle and not self.roca_detected): 
                if(self.theta>=angle):
                    self.twist.linear.x=0
                    self.twist.angular.z=0
                    self.pub_cmd.publish(self.twist)
                    break
                else:
                    
                    self.twist.linear.x=0
                    self.twist.angular.z=(abs(self.theta - 0) * (self.angular_velocity - 0.08) / (2*math.pi - 0) + 0.08)
                    self.pub_cmd.publish(self.twist)

    def set_angle(self,x1,y1):
        angle=np.arctan2((y1-self.posya),x1-self.posxa)
        if angle<0:
            angle=abs(angle)+math.pi
        return angle
    
    def go_to(self,x1,y1):
        distance = np.sqrt(pow(x1-self.posxa,2)+pow(y1-self.posya,2))
        angle = self.set_angle(x1,y1)
        self.move_angle(angle)
        target_time = self.ODOM_DISTANCE*distance/self.velocity+rospy.get_time()

        print("Coordinates: ",self.x," ,",self.y)
        print("Target coordinates: ",x1," ,",y1)

        while(target_time>rospy.get_time() and not self.roca_detected):
            self.twist.linear.x=self.velocity
            self.twist.angular.z=0
            self.pub_cmd.publish(self.twist)
            if((self.x >x1-const.POSITION_ERROR and self.x<x1+const.POSITION_ERROR) and (self.y>y1-const.POSITION_ERROR and self.y<y1+const.POSITION_ERROR)):
                break
        if(self.roca_detected):
            rospy.wait_for_service('arm_routine')
            try: 
                arm_routine = rospy.ServiceProxy('arm_routine', arm_routine)
            except:
                print("Service not found")

            print("KEEP")
            time.sleep(5)
            print("CONTINUE")
            print("ANGLE ",self.theta)
            print("TANGLE ",angle)

            distance = np.sqrt(pow(x1-self.posxa,2)+pow(y1-self.posya,2))
            angle = self.set_angle(x1,y1)

            self.move_angle(angle)
            distance2 = np.sqrt(pow(self.posxa-self.x,2)+pow(self.posya-self.y,2))
            distance3 = np.sqrt(pow(x1-self.x,2)+pow(y1-self.y,2))
            if(distance2<distance):
                print("Correcting distance ")
                target_time2 = self.ODOM_DISTANCE*distance3/self.velocity+rospy.get_time()
                while(target_time2>rospy.get_time() and not self.roca_detected):
                    self.twist.linear.x=self.velocity
                    self.twist.angular.z=0
                    self.pub_cmd.publish(self.twist)
                    if((self.x >x1-const.POSITION_ERROR and self.x<x1+const.POSITION_ERROR) and (self.y>y1-const.POSITION_ERROR and self.y<y1+const.POSITION_ERROR)):
                        print("Distance corrected")
                        break

        self.arrived=True
        self.pub_go_to.publish(self.arrived)

        self.twist.linear.x=0
        self.twist.angular.z=0
        self.pub_cmd.publish(self.twist)

    def main(self):
        self.routine("route")
        while not rospy.is_shutdown():
            self.count = 0
            for coordinates in self.coordinates:
                print("x: ",coordinates[0])
                print("y: ",coordinates[1])
                print("____________________routine_________________")
                self.go_to(coordinates[0],coordinates[1])
                self.posxa=coordinates[0]
                self.posya=coordinates[1]
                self.count+=1
            print("Arrived at destination")
            break
            self.rate.sleep()
    


if __name__=="__main__":
    route = Route()
    route.main()
