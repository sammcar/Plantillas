//--------------------- Librerías estándar y ROS ---------------------//

#include "ros/ros.h"  // API principal de ROS
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

#include <sstream>   // Para crear cadenas de texto de forma segura
#include <vector>    // Para manejo de arreglos dinámicos
#include <memory>    // Para punteros inteligentes (aunque no son necesarios aquí, son recomendados en C++ moderno)

//--------------------- Clase del Nodo ---------------------//

class EjemploNodo
{
public:
  // Constructor: se ejecuta al crear una instancia del nodo
  EjemploNodo()
  {
    // ROS NodeHandle: interfaz para interactuar con el sistema ROS
    // 'nh_' es una convención de nombre usada para objetos de tipo NodeHandle
    // En C++ moderno se usa inicialización en lista o en constructor

    // --------------------- Publicadores ---------------------
    // advertise<TipoMensaje>("nombre_topico", tamaño_buffer)
    publisher_string_ = nh_.advertise<std_msgs::String>("/string", 10);
    publisher_float_ = nh_.advertise<std_msgs::Float64MultiArray>("/Float", 10);

    // --------------------- Suscriptores ---------------------
    // subscribe<TipoMensaje>("nombre_topico", tamaño_buffer, función_callback)
    // El tercer parámetro es un puntero a función miembro (&Clase::función, this)
    subscriber_1_ = nh_.subscribe("/sucripcion1", 10, &EjemploNodo::callback1, this);
    subscriber_2_ = nh_.subscribe("/suscripcion2", 10, &EjemploNodo::callback2, this);

    // --------------------- Timer ---------------------
    // Crea un temporizador que llama a 'funcionTimer' cada 1.0 segundos
    // ros::Duration es una clase que representa tiempo (float o int)
    timer_ = nh_.createTimer(ros::Duration(1.0), &EjemploNodo::funcionTimer, this);

    // --------------------- Inicialización de variables ---------------------
    // Inicialización directa con C++11 (puedes usar = o llaves {})
    variable_ = 0.0;
  }

private:
  //--------------------- Atributos privados ---------------------//

  ros::NodeHandle nh_;  // Interfaz principal con el sistema de nodos ROS

  // Publicadores para enviar mensajes
  ros::Publisher publisher_string_;
  ros::Publisher publisher_float_;

  // Suscriptores para recibir mensajes
  ros::Subscriber subscriber_1_;
  ros::Subscriber subscriber_2_;

  // Temporizador (como un hilo que llama periódicamente a una función)
  ros::Timer timer_;

  // Variable interna de ejemplo
  double variable_;

  //--------------------- Funciones Callback ---------------------//

  // Función que se ejecuta cuando llega un mensaje al tópico /sucripcion1
  void callback1(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    // ConstPtr es un typedef de boost::shared_ptr<const T>, compatible con C++ moderno
    ROS_INFO_STREAM("Datos recibidos en suscripcion1: " << msg->data[0]);
  }

  // Función que se ejecuta cuando llega un mensaje al tópico /suscripcion2
  void callback2(const std_msgs::Float64MultiArray::ConstPtr& msg)
  {
    ROS_INFO_STREAM("Datos recibidos en suscripcion2: " << msg->data[0]);
  }

  // Función periódica que se ejecuta cada 1.0 segundos gracias al timer
  void funcionTimer(const ros::TimerEvent&)
  {
    std_msgs::String msg;
    msg.data = "Hola Mundo xd";  // Inicialización directa de string (C++11 compatible)

    publisher_string_.publish(msg);
    ROS_INFO_STREAM("Mensaje publicado: " << msg.data);
  }
};

//--------------------- Función principal ---------------------//

int main(int argc, char** argv)
{
  // Inicializa el nodo ROS (obligatorio en todo nodo)
  ros::init(argc, argv, "ejemplo_cpp");

  // Instancia del nodo, constructor hace todo el trabajo
  EjemploNodo nodo;

  // ros::spin() mantiene el nodo corriendo (como un bucle principal)
  ros::spin();

  return 0;  // Fin del programa
}
