#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from Tkinter import *

def enviar_a():
    pub.publish("a")

def enviar_b():
    pub.publish("b")

if __name__ == '__main__':
    rospy.init_node("enviar_comando")
    pub = rospy.Publisher('/enviar_comando', String, queue_size=10)  

    master = Tk()
    master.title("Configuracion Inicial")
    Button(master, text='Escanear', command=enviar_a).pack()
    Button(master, text='Salir', command=enviar_b).pack()

    master.mainloop()  