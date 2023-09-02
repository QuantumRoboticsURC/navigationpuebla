#!/usr/bin/env python3
import numpy as np
import rospy
import time
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Imu
from nav_helpers import nav_functions
import cv2
import math

class Map():
    def __init__(self):
        rospy.init_node("Mapping",anonymous=True)
        self.odom = Pose2D()
        self.imu_msg = Imu()
        self.listener_odom = rospy.Subscriber("/odometry",Pose2D,self.callbackOdom)
        self.listener_imu = rospy.Subscriber("/imu/data",Imu,self.callbackImu)
        self.listener_odom
        self.listener_imu
        self.rate = rospy.Rate(30)
        self.x = None
        self.y = None
        self.theta = None
        self.quatx = None
        self.quaty = None
        self.quatz = None
        self.quatw = None
        self.euler_roll = 0
        self.euler_pitch = 0
        self.euler_yaw = 0
        self.angle_degrees_x = 0
        self.angle_degrees_y = 0
        self.angle_degrees_z = 0
    
    def callbackOdom(self,data):
        self.x = data.x
        self.y = data.y
        self.theta = data.theta
    def callbackImu(self, data):
        self.quatx = data.orientation.x
        self.quaty = data.orientation.y
        self.quatz = data.orientation.z
        self.quatw = data.orientation.w
 
    def getangle(self):
        self.euler_roll, self.euler_pitch, self.euler_yaw = nav_functions.euler_from_quaternion([self.quatx, self.quaty, self.quatz, self.quatw])
        
        self.angle_degrees_x = math.degrees(np.arctan(self.euler_roll))
        self.angle_degrees_y = math.degrees(np.arctan(self.euler_pitch))
        self.angle_degrees_z = math.degrees(np.arctan(self.euler_yaw))
    
    def mapping(self):

        self.getangle()
    
        listax = []
        listay = []

        matrix1 = np.zeros([8,8])

        matrix2 = np.zeros([8,8])

        while True:
            state = int(self.x)
            state2 = int(self.y)

            while (state+1 >= self.x and state2+1 >= self.y):
                listax.append(self.angle_degrees_x)
                listay.append(self.angle_degrees_y)
        
        
            promx = sum(listax)/len(listax)
            promy = sum(listax)/len(listax)
    
            matrix1[state,state2] = promx
            matrix2[state,state2] = promy

            matrixacum1 = matrix1
            matrixacum2 = matrix2

            listax.clear()
            listay.clear()
            for i in range(len(matrix1)):
                for j in range(len(matrix1)):
                    if j == 0:
                        matrixacum1[i,j] = matrix1[i,j]
                    else:
                        matrixacum1[i,j] = matrix1[i,j]+ matrix1[i,j-1]
            #print(matrixacum1)

            for i in range(len(matrix2)):
                for j in range(len(matrix2)):
                    if j == 0:
                        matrixacum2[i,j] = matrix2[i,j]
                    else:
                        matrixacum2[i,j] = matrix2[i,j]+ matrix2[i,j-1]

            #print(matrixacum2)

            matrixfin = np.zeros([8,8])

            for i in range(len(matrix1)):
                for k in range(len(matrix1)):
                    if(matrixacum1[i,k] <= matrixacum2[i,k]):
                        matrixfin[i,k] = matrixacum1[i,k]
                    else:
                        matrixfin[i,k] = matrixacum2[i,k]

            print(matrixfin)

            #colors = np.array([[0,44,110],[0,54,135],[0,64,160],[0,74,186],[0,84,211],[51,118,200],[102,154,229],[153,187,237]],dtype = np.uint8)

            def matrix_color_map(matrix):

                colors = np.array([[0,44,110],[0,54,135],[0,64,160],[0,74,186],[0,84,211],[51,118,200],[102,154,229],[153,187,237]],dtype = np.uint8)
    
                matrix_map = np.zeros([800,800,3], dtype = np.uint8)

                for n in range(8):
        
                    for j in range(8):
                        if(matrix[n,j] >= 30):
                            matrix_map[n*100:n*100+100,j*100:j*100+100] = colors[0]
                        elif(matrix[n,j] < 30 and matrix[n,j] >= 22.5):
                            matrix_map[n*100:n*100+100,j*100:j*100+100] = colors[1]
                        elif(matrix[n,j] < 22.5 and matrix[n,j] >= 15):
                            matrix_map[n*100:n*100+100,j*100:j*100+100] = colors[2]
                        elif(matrix[n,j] < 15 and matrix[n,j] >= 7.5):
                            matrix_map[n*100:n*100+100,j*100:j*100+100] = colors[3]
                        elif(matrix[n,j] < 7.5 and matrix[n,j] >= 0):
                            matrix_map[n*100:n*100+100,j*100:j*100+100] = colors[4]
                        elif(matrix[n,j] < 0 and matrix[n,j] >= -7.5):
                            matrix_map[n*100:n*100+100,j*100:j*100+100] = colors[5]
                        elif(matrix[n,j] < -7.5 and matrix[n,j] >= -15):
                            matrix_map[n*100:n*100+100,j*100:j*100+100] = colors[6]
                        elif(matrix[n,j] < -15 and matrix[n,j] >= -22.5):
                            matrix_map[n*100:n*100+100,j*100:j*100+100] = colors[7]
                        elif(matrix[n,j] < -22.5):
                            matrix_map[n*100:n*100+100,j*100:j*100+100] = colors[7]
            
                return matrix_map
            matrixview = matrix_color_map(matrixfin)

            cv2.imshow('mapa',matrixview)

            if cv2.waitKey(1) & 0xFF == ord('s'):
                break
            #if cv2.waitKey(1)==ord('q'):
        cv2.destroyAllWindows()
    
    def main(self):

        while not rospy.is_shutdown():
            if (self.x is not None and self.y is not None and self.quatx is not None and self.quaty is not None and self.quatz is not None and self.quatw is not None):
                self.mapping()
            self.rate.sleep()


if __name__=="__main__":
    mapp = Map()
    mapp.main()
    
