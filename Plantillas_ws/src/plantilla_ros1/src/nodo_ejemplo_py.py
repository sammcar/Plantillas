#!/usr/bin/env python3

# El comentario de arriba es un shebang, le dice al sistema que use Python 3 al ejecutar este script

#------------------------- Librerías --------------------------------------#

# Se importan librerías estándar
import matplotlib.pyplot as plt
import math
import numpy as np

# Se importan librerías de ROS 1
import rospy
from std_msgs.msg import Float64MultiArray, String

#------------------------- Crear Nodo --------------------------------------#

class EjemploNodo:

    def __init__(self):
        # Inicializa el nodo con el nombre 'ejemplo'
        rospy.init_node('ejemplo', anonymous=True)

        # Publicadores
        # rospy.Publisher(nombreTopico, TipoMensaje, queue_size)
        self.publisher_string = rospy.Publisher('/string', String, queue_size=10)
        self.publisher_float = rospy.Publisher('/Float', Float64MultiArray, queue_size=10)

        # Suscriptores
        # rospy.Subscriber(nombreTopico, TipoMensaje, función_callback)
        rospy.Subscriber('/sucripcion1', Float64MultiArray, self.subscriptions1)
        rospy.Subscriber('/suscripcion2', Float64MultiArray, self.subscriptions2)

        # Variables internas
        self.variable = None

        # Temporizador (equivalente al timer de ROS 2)
        # rospy.Timer(duración, función)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.funcion_timer)

    #------------------------- Funciones Auxiliares --------------------------------------#

    def funcion_timer(self, event):
        # Publicar un mensaje tipo String
        msg = String()
        msg.data = "Hola Mundo xd"
        self.publisher_string.publish(msg)
        rospy.loginfo(f"Mensaje publicado: {msg.data}")

    def subscriptions1(self, msg):
        datos = msg.data
        rospy.loginfo(f"Datos recibidos en suscripcion1: {datos}")

    def subscriptions2(self, msg):
        datos = msg.data
        rospy.loginfo(f"Datos recibidos en suscripcion2: {datos}")

#------------------------- M A I N --------------------------------------#

if __name__ == '__main__':
    try:
        nodo = EjemploNodo()
        rospy.spin()  # Mantiene el nodo activo escuchando
    except rospy.ROSInterruptException:
        pass
