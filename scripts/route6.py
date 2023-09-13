#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import String,Bool, Int32
from navigationpuebla.msg import odom,target
import time
import consts as const
import math

class Route():
    def __init__(self):
        #En este constructor, se inicializan todas las variables, publicadores y subscriptores
        # Se inicia el nodo 
        rospy.init_node("Route",anonymous=True)

        # Se crea el mensaje de tipo twist
        self.twist = Twist()
        self.pub_cmd = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        #Publicador de una bandera booleana
        self.pub_go_to = rospy.Publisher("/go_to",Bool,queue_size=10)
        #Publicador del servo de la caja
        self.servocaja = rospy.Publisher("/servo_left",Int32,queue_size=10)

        #Se suscribe a la odometria recibida de odometry, y de una badera boolena
        rospy.Subscriber("/odometry",Pose2D,self.callback)
        rospy.Subscriber("/deteccion_roca",Bool,self.callback2)

        #Se inicaliza la variables
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
        self.ODOM_DISTANCE = const.ODOM_DISTANCE_CORRECTION
        self.posxa=self.x
        self.posya=self.y
        self.control = 0
        self.count = 0
    
    def callback(self,data):
        #Se escuchan los datos de odometry
        self.x=data.x
        self.y=data.y
        self.theta=data.theta
    
    def callback2(self,data):
        #Se escucha la bandera booleana 
        self.roca_detected=data.data

    def routine(self,param):
        #Se escoge la ruta que se va a ejecutar  
        if(param=="line"):
            self.coordinates=[(4,0)]
        elif(param=="angle45"):
            self.coordinates=[(3,3)]
        elif(param=="angle90"):
            self.coordinates=[(0,3)]
        elif(param=="zig"):
            self.coordinates=[(0,3),(3,0)]
        elif(param=="route"):
            #self.coordinates = [(6,0),(6,1),(0.5,1),(0.5,3),(6,3),(6,5)]
            #Ruta de 10*10 m^2 
            self.coordinates =[(8,0),(8,1),(0.5,1),(0.5,2),(8,2),(8,3),(0.5,3),(0.5,4),(8,4),(8,5),(0.5,5),(0.5,6),(8,6),(8,7),(0.5,7),(0.5,8),(8,8),(8,0),(0,0)]
            #self.coordinates = [(1,7),(2,7),(2,1),(3,1),(3,7),(4,7),(4,1),(5,1),(5,7),(6,7),(6,1),(7,1),(7,7)]
        else:
            print("Default")
            self.coordinates = [(1,1)]
    
    def move_angle(self,angle):
        #Se mueve al angulo y decide a donde girar 
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
                    #Se calcula la velocidad dependiendo de lo que le hace falta girar 
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
                    #Se calcula la velocidad dependiendo de lo que le hace falta girar 
                    self.twist.angular.z=(abs(self.theta - 0) * (self.angular_velocity - 0.08) / (2*math.pi - 0) + 0.08)
                    self.pub_cmd.publish(self.twist)

    def set_angle(self,x1,y1):
        #Se calcula el angulo objetivo
        angle=np.arctan2((y1-self.posya),x1-self.posxa)
        if angle<0:
            angle=abs(angle)+math.pi
        return angle
    
    def go_to(self,x1,y1):
        #Se calcula la distancia y cuanto se va a tener que girar
        distance = np.sqrt(pow(x1-self.posxa,2)+pow(y1-self.posya,2))
        angle = self.set_angle(x1,y1)
        #Gira
        self.move_angle(angle)
        #El tiempo que teoricamente se va tardar en llegar a la posicion 
        target_time = self.ODOM_DISTANCE*distance/self.velocity+rospy.get_time()

        print("Coordinates: ",self.x," ,",self.y)
        print("Target coordinates: ",x1," ,",y1)

        #Se va a avanzar al punto al que tiene que llegar
        while(target_time>rospy.get_time() and not self.roca_detected):
            self.twist.linear.x=self.velocity
            self.twist.angular.z=0
            self.pub_cmd.publish(self.twist)
            #En caso de que se detecte que el rover esta dentro de los margenes de error aceptables para x,y se detiene el movimiento.
            if((self.x >x1-const.POSITION_ERROR and self.x<x1+const.POSITION_ERROR) and (self.y>y1-const.POSITION_ERROR and self.y<y1+const.POSITION_ERROR)):
                break
            #Si la roca fue detectada en el deteccion7.py, route se pone en pausa por  90 segundos para darle tiempo a la brazo de recoger y al robot de centrarse
        if(self.roca_detected):
            time.sleep(90)
            
            print("KEEP")
            print("CONTINUE")
            print("ANGLE ",self.theta)
            print("TANGLE ",angle)

            #Una vez termina la pausa, vuelve a calcular la distancia
            distance = np.sqrt(pow(x1-self.posxa,2)+pow(y1-self.posya,2))
            #Vuelve a corregir el angulo. 
            angle = self.set_angle(x1,y1)
            self.move_angle(angle)

            distance2 = np.sqrt(pow(self.posxa-self.x,2)+pow(self.posya-self.y,2))
            distance3 = np.sqrt(pow(x1-self.x,2)+pow(y1-self.y,2))
            #revisa si el rover aun no ha llegado a su distancia objetivo 
            if(distance2<distance):
                print("Correcting distance ")
                #calcula el tiempo que le tomaria llegar a la posicion  
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
        time.sleep(5)
        #Especifica la ruta que va a ejectutar
        self.routine("route")
        while not rospy.is_shutdown():
            self.count = 0
            #Mueve el servo de la caja 
            #self.servocaja.publish(120)
            for coordinates in self.coordinates:
                #va moviendose a cada coordenada dentro de la ruta
                print("x: ",coordinates[0])
                print("y: ",coordinates[1])
                print("____________________routine_________________")
                #Manda el rover a la posicion deseada
                self.go_to(coordinates[0],coordinates[1])
                self.posxa=coordinates[0]
                self.posya=coordinates[1]
                self.count+=1
            print("Arrived at destination")
            break
            self.rate.sleep()
        #self.servocaja.publish(0)

    


if __name__=="__main__":
    # Se inicializa la el objeto de la clase route y se ejecuta su funcion
    route = Route()
    route.main()
