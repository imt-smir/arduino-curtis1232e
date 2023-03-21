/* Configurações básicas do robolo */
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"

/*
 * Constantes de uso geral 
 */
#define RAIO_RODA 6.25/100.0 // metros
#define DISTANCIA_ENTRE_RODAS 43.6/100.0 // metros
#define RADIANOS_PARA_ROT 0.1592
#define PERIODO_LEITURA 100 // 1x10^-3 s
#define MILLIS_PARA_S 1.0/1000.0


typedef float vel_t;

/* Funções cinemática */
void transforma_cinematica_robo(vel_t linear, vel_t angular, vel_t* mot1, vel_t* mot2);

/* Funções ROS */
void teleop_callback(const geometry_msgs::Twist& vel);



vel_t teleop_linear;
vel_t teleop_angular;
