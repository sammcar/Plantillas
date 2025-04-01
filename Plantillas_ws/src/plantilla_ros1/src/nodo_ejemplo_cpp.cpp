// ejemplo_nodo.cpp
// Nodo ROS 1 estructurado con timers, suscriptores y publicadores estilo modular

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>
#include <sstream>
#include <memory>

class EjemploNodo {
public:
  EjemploNodo() {
    // Crear publicadores
    pub_string_ = nh_.advertise<std_msgs::String>("/string", 10);
    pub_float_ = nh_.advertise<std_msgs::Float64MultiArray>("/Float", 10);

    // Crear suscriptores
    sub_1_ = nh_.subscribe("/sucripcion1", 10, &EjemploNodo::callback1, this);
    sub_2_ = nh_.subscribe("/suscripcion2", 10, &EjemploNodo::callback2, this);

    // Crear temporizador
    timer_ = nh_.createTimer(ros::Duration(1.0), &EjemploNodo::callbackTimer, this);

    // Inicializar variables
    variable_ = 0.0;
  }

private:
  ros::NodeHandle nh_;                      // Manejador del nodo
  ros::Publisher pub_string_;               // Publicador de tipo String
  ros::Publisher pub_float_;                // Publicador de tipo Float64MultiArray
  ros::Subscriber sub_1_;                   // Suscriptor 1
  ros::Subscriber sub_2_;                   // Suscriptor 2
  ros::Timer timer_;                        // Temporizador periódico

  double variable_;                         // Variable interna de ejemplo

  // Callback del temporizador (publica String cada 1 segundo)
  void callbackTimer(const ros::TimerEvent&) {
    std_msgs::String msg;
    msg.data = "Hola Mundo xd";
    pub_string_.publish(msg);
    ROS_INFO("Mensaje publicado: %s", msg.data.c_str());
  }

  // Callback del primer suscriptor
  void callback1(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    ROS_INFO("Datos recibidos en suscripcion1: %f", msg->data[0]);
  }

  // Callback del segundo suscriptor
  void callback2(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    ROS_INFO("Datos recibidos en suscripcion2: %f", msg->data[0]);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "ejemplo_cpp");
  EjemploNodo nodo;
  ros::spin();
  return 0;
}
