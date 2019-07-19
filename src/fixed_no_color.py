#!/usr/bin/env python

import rospy
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import numpy as np
import time
import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from Tkinter import *

pos_inicial = 3.2
pos_final = 1.7
filas = 290
datos_laser_final = np.zeros((filas, 726))
X_final = np.zeros((filas, 726))
Y_final = np.zeros((filas, 726))
Z_final = np.zeros((filas, 726))
Zi = 0
Xo = np.zeros((filas, 726))
Yo = np.zeros((filas, 726))
Zo = np.zeros((filas, 726))

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
        rospy.loginfo("Starting node exapmple")
        rospy.Subscriber('/scan', LaserScan, self.read_Hokuyo) # 10Hz
        rospy.Subscriber('/motor_controller/state', JointState, self.read_Dynamixel) # 20Hz
        rospy.spin()

    def read_Dynamixel(self,data):
        self.phi = data.current_pos - 2.62   # Offset del servo
        #EL 0 es 2.62 radianes
        # res 0.29 # degrees
        # pos horizontal = 2.62 rad

        
    def read_Hokuyo(self,data):
        #print(datetime.datetime.now())
        
        self.veces = self.veces + 1

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
        #Zi = self.veces

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

        # Split XYZ de la matriz P0
        Xo[self.veces-1,:] = XYZ_0[0,:]
        Yo[self.veces-1,:] = XYZ_0[1,:]
        Zo[self.veces-1,:] = XYZ_0[2,:]

        pos.data = pos_inicial - (self.veces*0.0051)
        pub.publish(pos)

        if self.veces == filas-1:
            fig = plt.figure()
            ax = Axes3D(fig)
            X=np.array(Xo)
            Y=np.array(Yo)
            Z=np.array(Zo)
            np.savetxt('Xo.txt', Xo)
            np.savetxt('Yo.txt', Yo)
            np.savetxt('Zo.txt', Zo)
            #colorMap = 100.0*((Z-Z.min)/(Z.max-Z.min))
            #scf = ax.scatter(X, Y, Z, marker='.', cmap='jet',c=np.int(colorMap), s= 0.1)
            scf = ax.scatter(-X, Y, -Z, marker='.', cmap='jet', s= 0.1)
            plt.xlabel('X')
            plt.ylabel('Y')
            plt.savefig('/home/ros/Escritorio/HOKUYO/prueba hokuyo/reconstruccion.png')
            plt.show()
            rospy.signal_shutdown("Tiempo Terminado")
            f.close            


def mover_inicio():
    pub.publish(pos_inicial)

def mover_final():
    pub.publish(pos_final)


if __name__ == '__main__':
    rospy.init_node("Recons_dinam_no_move")
    pub = rospy.Publisher('/motor_controller/command', Float64, queue_size=10)
    pos = Float64()

    master = Tk()
    master.title("Configuracion Inicial")
    Button(master, text='Mover Inicio', command=mover_inicio).pack()
    Button(master, text='Mover Final', command=mover_final).pack()
    master.mainloop()

    #f = open('/home/ros/Escritorio/HOKUYO/prueba hokuyo/reconstruccion_puerta.txt', 'w')
    cv = First()
