#!/usr/bin/env python3.7

import rospy
import numpy as np
import pygame
import sys
from geometry_msgs.msg import Twist, Pose2D

class PygameSim():
    def __init__(self):
        pygame.init() # se inicia el nodo y los subscribers de cmd_vel, que no se utiliza, y odometry
        rospy.init_node("Pygame",anonymous=True)
        rospy.Subscriber("/odometry",Pose2D,self.callback)
        rospy.Subscriber("/cmd_vel",Twist,self.callback2)
        #odometry que se utiliza para la posicion
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        #Cmd_vel que no se utiliza
        self.vx=0.0
        self.vy=0.0
        self.vTheta=0.0

    def callback(self,data): # se gusrdan los datos de los suscribers odometry localmente
        self.x=data.x
        self.y=data.y
        self.theta=data.theta

    def callback2(self,data): # se gusrdan los datos de los suscribers cmd_vel localmente
        self.vx=data.linear.x*np.cos(self.theta)
        self.vy=data.linear.x*np.sin(self.theta)
        self.vTheta=data.angular.z

    def main(self): #este es el main
        cont = 0 
        img = pygame.image.load('arrow2.png') # se carga la imagen
        while not rospy.is_shutdown():
            x = float(self.x) #pose en x
            y = float(self.y) #pose en y
            theta = float(self.theta) #angulo
            angulo = np.degrees(theta)

            vx = float(self.vx) #velocidad en x
            vy = float(self.vy) #velocidad en y
            velTheta = float(self.vTheta) #velocidad de rotacion

            pygame.init() #inicia el pygame
            if cont==0: #si es la primer vuelta en el ciclo, crea la ventana. Esto lo hace solo 1 vez.
                ventana = pygame.display.set_mode((1000,1000))
                pygame.display.set_caption("Simulador odometry")
                cont = cont +1
            ventana.fill((0,255,0)) #llena la ventana de color verde cada iteración para borrar la imagen anterior
            #cuadrado = pygame.Rect(int(x*100),int(y*100),100,100)
            imgrot = pygame.transform.rotate(img, -angulo) #le da el valor de rotación a la imagen con base en el angulo de odometry
            ventana.blit(imgrot, (int(x*100),int(y*100))) #dibuja la imagen con base en las coordenadas de odometry
            clock=pygame.time.Clock() # crea la variable para la taza de actualización
            clock.tick(24) #define la actualización del código a 24fps. 24 veces por segundo
            #pygame.draw.rect(ventana,(255,0,0),cuadrado)
            pygame.display.update() #actualiza la pantalla para mostrar los nuevos dibujos

if __name__=="__main__": #se llama el main definido anteriormente en forma de objeto
    py = PygameSim()
    py.main()