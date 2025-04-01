// nodo_red_neuronal.cpp
// Nodo ROS 1 que simula una red neuronal simple de 2 neuronas
// con activación lineal y publica el estado de la neurona 1.

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <vector>

class NodoRedNeuronal {
public:
  NodoRedNeuronal() {
    // Crear publicador para el tópico /actividad_neuronal
    // Se publicará únicamente el valor actualizado de la neurona 1
    pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/actividad_neuronal", 10);

    // Crear un timer que ejecuta la función actualizarRed() cada 100 ms
    timer_ = nh_.createTimer(ros::Duration(dt_), &NodoRedNeuronal::actualizarRed, this);

    // Inicializar entradas (K1, K2) y estados iniciales de las neuronas
    K1_ = 2.1;
    K2_ = 2.0;
    n_ = {{0.0, 0.0}, {0.0, 0.0}};  // n[i][0] es el estado actual, n[i][1] es el próximo estado
  }

private:
  ros::NodeHandle nh_;                  // Nodo principal
  ros::Publisher pub_;                  // Publicador de mensajes
  ros::Timer timer_;                    // Temporizador para ejecución periódica

  // Parámetros de la red neuronal
  double dt_ = 0.1;                     // Paso de tiempo
  double tau_ = 1.0;                    // Constante de tiempo
  double u_ = 0.1;                      // Umbral constante
  double K1_, K2_;                      // Entradas externas a las neuronas
  double n_[2][2];                      // Estados de las neuronas (actual y siguiente)

  // Función de activación lineal con saturación inferior en 0
  double lineal(double X) {
    return std::max(0.0, X);
  }

  // Función llamada por el timer cada dt segundos
  void actualizarRed(const ros::TimerEvent&) {
    // Actualizar el estado de las dos neuronas según ecuación diferencial discretizada
    n_[0][1] = n_[0][0] + (dt_ / tau_) * (-n_[0][0] + lineal(K1_ - 3 * n_[1][0] - u_));
    n_[1][1] = n_[1][0] + (dt_ / tau_) * (-n_[1][0] + lineal(K2_ - 3 * n_[0][0] - u_));

    // Crear y llenar el mensaje con el valor actualizado de la neurona 1 solamente
    std_msgs::Float64MultiArray msg;
    msg.data.push_back(n_[0][1]);  // Solo neurona 1
    pub_.publish(msg);

    // Imprimir el valor de la neurona 1 en consola
    ROS_INFO("n1: %.3f", n_[0][1]);

    // Actualizar los estados actuales con los nuevos valores
    for (int i = 0; i < 2; ++i) {
      n_[i][0] = n_[i][1];
    }
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "nodo_red_neuronal");  // Inicializar el nodo ROS
  NodoRedNeuronal nodo;                        // Instanciar clase
  ros::spin();                                 // Mantener el nodo activo
  return 0;
}
