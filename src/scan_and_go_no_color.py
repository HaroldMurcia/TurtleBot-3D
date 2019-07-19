#!/usr/bin/env python

import rospy
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_msgs.msg import Empty
from kobuki_msgs.msg import Sound
import numpy as np
from time import time
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Tkinter import *

pos_inicial = 3.2
pos_final = 1.7
filas = 290


class First(): # Crea clase
    def __init__(self):
        self.phi =0
        self.tx_1=0
        self.ty_1=0
        self.tz_1=-0.09      #Distancia del centro del hokuyo al P1, eje de rotacion del servo

        self.tx_2=0
        self.ty_2=0
        self.tz_2=-0.048     #Distancia del eje de rotacion del servo a la base superior de la tortuga

        self.tx_0=0
        self.ty_0=0.06
        self.tz_0=-0.422     #Distancia de la base superior de la tortuga al punto 0, o al piso

        self.angle_min = 0
        self.angle_max = 0
        self.ang_inc = 0
        self.theta = 0
        self.veces = 0
        self.Zinicial = 0
        self.ResAng = pos_inicial - pos_final
        self.enable = 0

        self.pos_x = 0
        self.pos_y = 0
    
        self.pos_ang_e = 0
        self.yaw = 0

        self.giro = 0

        self.itera = 0

        self.Xo = np.zeros((726))
        self.Yo = np.zeros((726))
        self.Zo = np.zeros((726))

        self.phi = 3.0 - 2.62  #Posicion del servo en radianes

        rospy.loginfo("Escaneando...")
        rospy.Subscriber('/scan', LaserScan, self.read_Hokuyo) # 10Hz
        rospy.Subscriber('/enviar_comando', String, self.read_Command)
        rospy.Subscriber('/odom', Odometry, self.read_Odom)
        #rospy.Subscriber('/odometry/filtered', Odometry, self.read_Odom)
        rospy.spin()

    def read_Odom(self,data):
        self.pos_x = data.pose.pose.position.x
        self.pos_y = data.pose.pose.position.y
        quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        self.pos_ang_e = euler_from_quaternion(quaternion)
        self.yaw = -self.pos_ang_e[2]          

    def read_Command(self,data):
        #print data.data
        if data.data == 'b':
            X=np.array(self.Xo)
            Y=np.array(self.Yo)
            Z=np.array(self.Zo)

            #Desde la posicion 727

            #recons = np.array([ [X, Y, Z] ])
            for cont in range(0, len(X)):
                f.write("%f\t" %X[cont])
                f.write("%f\t" %Y[cont])
                f.write("%f\n" %Z[cont])

            np.savetxt('/home/ros/Escritorio/HOKUYO/prueba hokuyo/Reconstruccion_din_Ang_fijo/X.xls', X)
            np.savetxt('/home/ros/Escritorio/HOKUYO/prueba hokuyo/Reconstruccion_din_Ang_fijo/Y.xls', Y)
            np.savetxt('/home/ros/Escritorio/HOKUYO/prueba hokuyo/Reconstruccion_din_Ang_fijo/Z.xls', Z)
            f.close
            rospy.signal_shutdown("Tiempo Terminado")

        
    def read_Hokuyo(self,data):      
        #self.veces = self.veces + 1            
        #rospy.loginfo("reading LiDAR")
        datos_laser = np.asarray(data.ranges)
        self.angle_min= data.angle_min
        self.angle_max= data.angle_max
        self.N = len(data.ranges)
        self.ang_inc = (self.angle_max-self.angle_min)/self.N
        self.theta = np.linspace(self.angle_min, self.angle_max, self.N)

        Xi = datos_laser*np.sin(self.theta)
        Yi = datos_laser*np.cos(self.theta)
        Zi = datos_laser*0

        # Transformacion cinematica (Traslacion) a P1
        A = np.array([ [1,0,0,self.tx_1],[0,1,0,self.ty_1],[0,0,1,self.tz_1],[0,0,0,1] ])
        B = np.ones([4,self.N])
        B[0,:] = Xi
        B[1,:] = Yi
        B[2,:] = Zi
        XYZ_1 = np.matmul(A, B)

        # Transformacion cinematica (Rotacion) a P1
        C = np.array([ [1,0,0,0], [0,np.cos(self.phi),-np.sin(self.phi),0], [0,np.sin(self.phi),np.cos(self.phi),0], [0,0,0,1] ])
        XYZ_0 = np.matmul(C, XYZ_1)

        # Transformacion cinematica (Traslacion) a P2
        D = np.array([ [1,0,0,self.tx_2],[0,1,0,self.ty_2],[0,0,1,self.tz_2],[0,0,0,1] ])
        XYZ_0 = np.matmul(D, XYZ_0)

        # Transformacion cinematica (Traslacion) a P0
        E = np.array([ [1,0,0,self.tx_0],[0,1,0,self.ty_0],[0,0,1,self.tz_0],[0,0,0,1] ])
        XYZ_0 = np.matmul(E, XYZ_0)

        # Transformacion cinematica (Traslacion) a P0 por movimiento del robot
        F = np.array([ [1,0,0,self.pos_y],[0,1,0,np.absolute(self.pos_x)],[0,0,1,0],[0,0,0,1] ])
        XYZ_0 = np.matmul(F, XYZ_0)

        # Transformacion cinematica (Rotacion) a Po por giro del robot sobre el eje Z
        G = np.array([ [np.cos(self.yaw),-np.sin(self.yaw),0,0], [np.sin(self.yaw),np.cos(self.yaw),0,0], [0,0,1,0], [0,0,0,1] ])
        XYZ_0 = np.matmul(G, XYZ_0)

        # Extrayendo valores de X, Y, Z
        self.Xo = np.concatenate((self.Xo, XYZ_0[0,:]))
        self.Yo = np.concatenate((self.Yo, XYZ_0[1,:]))
        self.Zo = np.concatenate((self.Zo, XYZ_0[2,:]))

        #print ("Escaneo #: %i" %self.veces)
        #sonido.publish(1)

if __name__ == '__main__':
    try:        
        rospy.init_node("Recons_ang_fixed_no_color")
        pub = rospy.Publisher('/motor_controller/command', Float64, queue_size=10)
        reset = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        reset.publish(Empty())
        sonido = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
        pos = Float64()
        f = open('/home/ros/Escritorio/HOKUYO/prueba hokuyo/Reconstruccion_din_Ang_fijo/Recons_Ang_Fijo.txt', 'w')

        cv = First()
    except rospy.ROSInterruptException:
        f.close   
