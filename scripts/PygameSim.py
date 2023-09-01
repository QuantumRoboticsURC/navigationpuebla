#!/usr/bin/env python3.7

import rospy
import numpy as np
import pygame
import sys
from geometry_msgs.msg import Twist, Pose2D

class PygameSim():
    def __init__(self):
        pygame.init()
        rospy.init_node("Pygame",anonymous=True)
        rospy.Subscriber("/odometry",Pose2D,self.callback)
        rospy.Subscriber("/cmd_vel",Twist,self.callback2)
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        #Cmd_vel
        self.vx=0.0
        self.vy=0.0
        self.vTheta=0.0

    def callback(self,data):
        self.x=data.x
        self.y=data.y
        self.theta=data.theta

    def callback2(self,data):
        self.vx=data.linear.x*np.cos(self.theta)
        self.vy=data.linear.x*np.sin(self.theta)
        self.vTheta=data.angular.z

    def main(self): #este es el main
        cont = 0
        coord = (100,100)
        while not rospy.is_shutdown():
            x = float(self.x) #pose en x
            y = float(self.y) #pose en y
            theta = float(self.theta) #angulo
            vx = float(self.vx) #velocidad en x
            vy = float(self.vy) #velocidad en y
            velTheta = float(self.vTheta) #velocidad de rotacion

            pygame.init()
            if cont==0:
                ventana = pygame.display.set_mode((800,800))
                ventana.fill((0,255,0))
                cont = cont +1
            cuadrado = pygame.Rect(coord[0],coord[1],100,100)
            #clock = pygame.time.Clock()
            coord = (coord[0]+10,coord[1]+10)
            pygame.draw.rect(ventana,(255,0,0),cuadrado)
            pygame.display.update()

if __name__=="__main__":
    py = PygameSim()
    py.main()