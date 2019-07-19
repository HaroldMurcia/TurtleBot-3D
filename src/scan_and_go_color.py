#!/usr/bin/env python

import rospy
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Empty
import numpy as np

from kobuki_msgs.msg import Sound
import time
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Tkinter import *
from time import time

from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

pos_inicial = 3.05
pos_final = 2.28
pos_centro = 2.62
filas = 151
cv2_img = 0
R_t1 = []
G_t1 = []
B_t1 = []

R_t1.append(0)
G_t1.append(0)
B_t1.append(0)

# Instantiate CvBridge
bridge = CvBridge()

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
        self.theta1 = 0
        self.theta = 0
        self.center = 0
        self.veces = 0
        self.Zinicial = 0
        self.ResAng = pos_inicial - pos_final

        self.pos_x = 0
        self.pos_y = 0
    
        self.pos_ang_e = 0
        self.yaw = 0

        self.Xo = np.zeros((1))
        self.Yo = np.zeros((1))
        self.Zo = np.zeros((1))
        self.R_t = np.zeros((1))
        self.G_t = np.zeros((1))
        self.B_t = np.zeros((1))

        self.R = np.zeros((1))
        self.G = np.zeros((1))
        self.B = np.zeros((1))

        self.pixel_x = 0    #filas
        self.pixel_y = 0    #columnas
        self.pixel_x1 = 0    #filas
        self.pixel_y1 = 0    #columnas
        self.pixel_y2 = 0    #columnas

        self.pix_in = 310
        self.pix_end = 467

        self.enable = 0
        self.once = 0
        self.channel_b = 0
        self.channel_g = 0
        self.channel_r = 0
        self.foto_tomada = 0
        self.espera = 0
        self.giro = 0

        self.tiempo_inicial = 0
        self.tiempo_final = 0
        self.tiempo_ejecucion = 0

        self.phi = 3.0 - 2.62  #Posicion del servo en radianes

        rospy.loginfo("Starting node example")
        rospy.Subscriber('/scan', LaserScan, self.read_Hokuyo) # 10Hz
        rospy.Subscriber('/enviar_comando', String, self.read_Command)
        # Set up your subscriber and define its callback
        rospy.Subscriber('/image_view/output', Image, self.take_photo)
        rospy.Subscriber('/odom', Odometry, self.read_Odom)
        #rospy.Subscriber('/odometry/filtered', Odometry, self.read_Odom)
        rospy.spin()

    def read_Odom(self,data):
        self.pos_x = data.pose.pose.position.x
        self.pos_y = data.pose.pose.position.y
        quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,-data.pose.pose.orientation.z,data.pose.pose.orientation.w)
        self.pos_ang_e = euler_from_quaternion(quaternion)
        self.yaw = self.pos_ang_e[2]        

    def take_photo(self,data):
        if self.espera == 0:
            print "Take Photo"
            cv2_img = bridge.imgmsg_to_cv2(data, "bgr8")
            #cv2.imwrite('/home/ros/Escritorio/HOKUYO/prueba hokuyo/reconstruccion_color/image.jpeg', cv2_img)

            #b,g,r = cv2.split(cv2_img)
            (self.channel_b, self.channel_g, self.channel_r) = (cv2_img[:,:,0], cv2_img[:,:,1], cv2_img[:,:,2])

            #(480,640)
            #print self.channel_b.shape
            #print self.channel_g.shape
            #print self.channel_r.shape

            self.foto_tomada = 1

            """if self.once == 1:
                rospy.signal_shutdown("Tiempo Terminado")
                f.close"""
        
    def read_Command(self,data):
        print data.data
        if data.data == 'b':
            X=np.array(self.Xo)
            Y=np.array(self.Yo)
            Z=np.array(self.Zo)

            #recons = np.array([ [X, Y, Z] ])
            for cont in range(0, len(X)):
                f.write("%f\t" %X[cont])
                f.write("%f\t" %Y[cont])
                f.write("%f\t" %Z[cont])
                f.write("%f\t" %R_t1[cont])
                f.write("%f\t" %G_t1[cont])
                f.write("%f\n" %B_t1[cont])

            np.savetxt('/home/ros/Escritorio/HOKUYO/prueba hokuyo/reconstruccion_color/Xo.xls', X)
            np.savetxt('/home/ros/Escritorio/HOKUYO/prueba hokuyo/reconstruccion_color/Yo.xls', Y)
            np.savetxt('/home/ros/Escritorio/HOKUYO/prueba hokuyo/reconstruccion_color/Zo.xls', Z)
            np.savetxt('/home/ros/Escritorio/HOKUYO/prueba hokuyo/reconstruccion_color/R.xls', R_t1)
            np.savetxt('/home/ros/Escritorio/HOKUYO/prueba hokuyo/reconstruccion_color/G.xls', G_t1)
            np.savetxt('/home/ros/Escritorio/HOKUYO/prueba hokuyo/reconstruccion_color/B.xls', B_t1)
            #self.once = 1
            f.close
            rospy.signal_shutdown("Tiempo Terminado")

    def read_Hokuyo(self,data):
        self.tiempo_inicial = time()
        if self.foto_tomada == 1:
            self.foto_tomada = 0
            self.espera = 1

            datos_laser = np.asarray(data.ranges)
            #Datos extraidos al ancho de la resolucion de la camara
            datos_laser_extraidos = np.asarray(datos_laser[self.pix_in:self.pix_end])
            self.angle_min= data.angle_min
            self.angle_max= data.angle_max
            self.N = len(datos_laser)
            self.center = self.N/2
            self.ang_inc = (self.angle_max-self.angle_min)/self.N
            self.theta1 = np.linspace(self.angle_min, self.angle_max, self.N)
            self.theta = np.asarray(self.theta1[self.pix_in:self.pix_end])

            self.N = len(datos_laser_extraidos)

            Xi = datos_laser_extraidos*np.sin(self.theta)
            Yi = datos_laser_extraidos*np.cos(self.theta)
            Zi = datos_laser_extraidos*0

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

            self.R_t = np.array(self.channel_r[125, :])
            self.G_t = np.array(self.channel_g[125, :])
            self.B_t = np.array(self.channel_b[125, :])

            print self.R_t.shape
            print self.G_t.shape
            print self.B_t.shape

            #Son 157 datos por escaneo, por lo tanto deben haber 157 pixeles por escaneo
            for self.pixel_y in range(0, 628):
                self.pixel_y1 = self.pixel_y1 + 1
                if self.pixel_y1 == 4:
                    R_t1.append(self.R_t[(628 - 1) - self.pixel_y])
                    G_t1.append(self.G_t[(628 - 1) - self.pixel_y])
                    B_t1.append(self.B_t[(628 - 1) - self.pixel_y])
                    self.pixel_y1 = 0

            print len(R_t1)
            print len(G_t1)
            print len(B_t1)

            self.espera = 0

            self.tiempo_final = time()
            self.tiempo_ejecucion = self.tiempo_final - self.tiempo_inicial
            print self.tiempo_inicial
            print self.tiempo_final
            print self.tiempo_ejecucion

if __name__ == '__main__':
    try:        
        rospy.init_node("Recons_ang_fixed_color")
        pub = rospy.Publisher('/motor_controller/command', Float64, queue_size=10)
        reset = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        reset.publish(Empty())
        sonido = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=10)
        pos = Float64()

        f = open('/home/ros/Escritorio/HOKUYO/prueba hokuyo/reconstruccion_color/color.txt', 'w')
        cv = First()
    except rospy.ROSInterruptException:
        f.close