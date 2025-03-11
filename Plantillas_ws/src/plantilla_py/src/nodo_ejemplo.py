#!/usr/bin/env python3

# El comentario de arriba es un shebang, le dice al entorno donde se encuentra la instalación de python

#------------------------- Librerías --------------------------------------#

# Se importan dependencias normales

import matplotlib.pyplot as plt
import math
import numpy as np

# Se importan dependencias de ROS

import rclpy # Libreria Python-Ros2
from rclpy.node import Node # Clase de nodo
from std_msgs.msg import Float64MultiArray, String # Se importan mensajes de los topicos que se usaran

class Ejemplo_Nodo(Node):

#------------------------- Crear Nodo --------------------------------------#
    def __init__(self):
        super().__init__('ejemplo')
        
        # Publicadores
        # self.nombrePublicador = self.create_publisher(TipoMensaje,'/nombreTopico',Frecuencia de Publicacion)
        self.publisherString = self.create_publisher(String, '/string', 10)
        self.publisherFloat = self.create_publisher(Float64MultiArray, '/Float', 10)

        # Suscriptores
        # self.create_subscription(TipoMensaje,'/nombreTopico', Funcion que se ejectura al actualizar del topico, Frecuencia de Actualizacion)
        self.create_subscription(Float64MultiArray, '/sucripcion1', self.subscriptions1, 100)
        self.create_subscription(Float64MultiArray, '/suscripcion2', self.subscriptions2, 100)

        # Variables
        self.variable = None

        # Funcion con Timer
        # nombreTimer = self.create_timer(Cada cuando se ejecuta el Timer, funcion que llama el timer)
        self.timer = self.create_timer(1.0, self.funcionTimer)

#------------------------- Funciones Auxiliares --------------------------------------#

    def funcionTimer(self):
        # Crear y publicar el mensaje String correctamente
        msg = String()
        msg.data = "Hola Mundo xd"
        self.publisherString.publish(msg)
        self.get_logger().info(f"Mensaje publicado: {msg.data}")

    def subscriptions1(self, msg):
        datos = msg.data
        self.get_logger().info(f"Datos recibidos en suscripcion1: {datos}")

    def subscriptions2(self, msg):
        datos = msg.data
        self.get_logger().info(f"Datos recibidos en suscripcion2: {datos}")

#------------------------- M A I N --------------------------------------#

def main(args=None):
    rclpy.init(args=args)
    node = Ejemplo_Nodo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()