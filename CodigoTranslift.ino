#include <Arduino.h>
#include "translift.h"
#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(53); // Pino CS do Módulo CAN

vel_t teleop_linear = 0.0; // Velocidade linear desejada do AGV (m/s)
vel_t teleop_angular = 0.0; // Velocidade angular desejada do AGV (rad/s)
vel_t motE_set_point, motD_set_point; // Set Points para as velocidades desejadas

long t = 0;

int tarefa_atual = 0;
int n_tarefas = 4;

int rpmLidoE = 0; // RPM lido no motor esquerdo
int rpmLidoD = 0; // RPM lido no motor direito

int acelerador_E = 0; // Porcentagem da velocidade aplicada no motor esq. em relação a velocidade máxima
int acelerador_D = 0; // Porcentagem da velocidade aplicada no motor dir. em relação a velocidade máxima

int maxspeed = 760; // Velocidade máxima do motor definida nos parâmetros do driver (RPM)
float transmissao = 60.0/(46*38); // Relação de transmissao entre o eixo do motor e a roda

long previousMillis = 0;

// Mensagens CAN

struct can_frame VCL_ThrottleE; // acelerador_E Motor Esquerdo
struct can_frame VCL_ThrottleD; // acelerador_D Motor Direito
struct can_frame MOTOR_SPEED_A_E; // Leitura RPM Esquerdo (Solicitação)
struct can_frame MOTOR_SPEED_A_D; // Leitura RPM Direito  (Solicitação)
struct can_frame MOTOR_SPEED_A_E_recv; // Leitura RPM Esquerdo (Recebimento)
struct can_frame MOTOR_SPEED_A_D_recv; // Leitura RPM Direito  (Recebimento)
struct can_frame canMsgReceive; // Receber mensagem CAN

// Inicialização rosserial

ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub_teleop("cmd_vel", teleop_callback);
tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);


// Variáveis de odometria

double dx = 0.0;
double dy = 0.0;
double dth = 0.0;
double dxy = 0.0;
double vx = 0.0;
double vth = 0.0; // NOVO
char base_link[] = "/base_link";
char odom[] = "/odom";
double two_pi = 6.28319;
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
double xy_pos = 0.0; // NOVO

void setup() {
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // BaudRate do CAN do driver
  mcp2515.setNormalMode();

  mensagensCAN(); // Declara o frame das mensagens CAN  

  nh.initNode();
  nh.subscribe(sub_teleop);
  broadcaster.init(nh);
  nh.advertise(odom_pub);
  
}

void loop() {
  nh.spinOnce(); // Sintaxe para iniciar o loop do ROS

  if(millis() - t > PERIODO_LEITURA/n_tarefas){

    if(tarefa_atual == 0){
      transforma_cinematica_robo(teleop_linear, teleop_angular, &motE_set_point, &motD_set_point); 
      Throttle_CAN(motE_set_point * 60.0/transmissao, motD_set_point * 60.0/transmissao); // Envia a velocidade desejada de cada motor em RPM
      tarefa_atual++;
    }

    else if(tarefa_atual == 1){
      mcp2515.sendMessage(&MOTOR_SPEED_A_E); // Envia a solicitação de leitura da velocidade do motor esq.
      tarefa_atual++;
    }
    
    else if(tarefa_atual == 2){
      mcp2515.sendMessage(&MOTOR_SPEED_A_D); // Envia a solicitação de leitura da velocidade do motor dir.
      tarefa_atual++;
    }

    else {
      envia_odometria(rpmLidoE * transmissao/60.0, rpmLidoD * transmissao/60.0);
      tarefa_atual = 0;
    }
    
    t = millis();
    
  }
  
  LerRPM();
  
}

void transforma_cinematica_robo(vel_t linear, vel_t angular, vel_t* mot1, vel_t* mot2) {
  *mot1 = (2 * linear - angular * DISTANCIA_ENTRE_RODAS) * RADIANOS_PARA_ROT / (2 * RAIO_RODA);
  *mot2 = (2 * linear + angular * DISTANCIA_ENTRE_RODAS) * RADIANOS_PARA_ROT / (2 * RAIO_RODA);
  // mot1 mot2 está em rotações por segundo
}

void teleop_callback(const geometry_msgs::Twist& vel) { // Função de callback que recebe do tópico "cmd_vel" a velocidade desejada pelo usuário
  teleop_linear = vel.linear.x;
  teleop_angular = vel.angular.z;
}


void envia_odometria(vel_t mot1_vel_atual, vel_t mot2_vel_atual) {
  
  mot1_vel_atual = mot1_vel_atual * two_pi * RAIO_RODA;
  mot2_vel_atual = mot2_vel_atual * two_pi * RAIO_RODA;

  /* Odometria */
  // Criando TF e publicando Odom

  dxy = (mot1_vel_atual + mot2_vel_atual) * (millis() - previousMillis) / (2 * 1000); // Distancia instantanea (modulo)
  dth = (-mot1_vel_atual + mot2_vel_atual) * (millis() - previousMillis) / (1000 * DISTANCIA_ENTRE_RODAS); // Rotação instantanea

  previousMillis = millis();

  xy_pos += dxy; // NOVO
  
  dx = cos(dth) * dxy;
  dy = sin(dth) * dxy;
  
  theta += dth;
  x_pos += (cos(theta) * dx - sin(theta) * dy);
  y_pos += (sin(theta) * dx + cos(theta) * dy);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);
  geometry_msgs::TransformStamped t;

  vx = (mot1_vel_atual + mot2_vel_atual) / 2;
  vth = (mot2_vel_atual - mot1_vel_atual) / (43.6/100.0);

  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = x_pos;
  t.transform.translation.y = y_pos;
  t.transform.translation.z = 0.0;
  t.transform.rotation = odom_quat;
  t.header.stamp = nh.now(); //talvez
  
  broadcaster.sendTransform(t);
  
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = "odom";
  odom_msg.pose.pose.position.x = x_pos;
  odom_msg.pose.pose.position.y = y_pos;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;
  
  odom_msg.child_frame_id = base_link;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = vth;
 
  odom_pub.publish(&odom_msg);
  
}


void LerRPM(){

  // Verifica se recebeu uma mensagem CAN
  
  if (mcp2515.readMessage(&canMsgReceive) == MCP2515::ERROR_OK) {

     // Verifica se a mensagem recebida corresponde a velocidade do motor esquerdo
     
     if ((canMsgReceive.can_id == MOTOR_SPEED_A_E_recv.can_id)&& 
       (canMsgReceive.can_dlc == MOTOR_SPEED_A_E_recv.can_dlc)&&
       (canMsgReceive.data[0] == MOTOR_SPEED_A_E_recv.data[0])&&
       (canMsgReceive.data[1] == MOTOR_SPEED_A_E_recv.data[1])&&
       (canMsgReceive.data[2] == MOTOR_SPEED_A_E_recv.data[2])&&
       (canMsgReceive.data[3] == MOTOR_SPEED_A_E_recv.data[3]))
       {
        // Une os valores dos bytes de dados 4 e 5 do frame para gerar o valor do RPM
        rpmLidoE = canMsgReceive.data[5]*256 + canMsgReceive.data[4];
      } 

      // Verifica se a mensagem recebida corresponde a velocidade do motor direito
      
      if ((canMsgReceive.can_id == MOTOR_SPEED_A_D_recv.can_id)&&
       (canMsgReceive.can_dlc == MOTOR_SPEED_A_D_recv.can_dlc)&&
       (canMsgReceive.data[0] == MOTOR_SPEED_A_D_recv.data[0])&&
       (canMsgReceive.data[1] == MOTOR_SPEED_A_D_recv.data[1])&&
       (canMsgReceive.data[2] == MOTOR_SPEED_A_D_recv.data[2])&&
       (canMsgReceive.data[3] == MOTOR_SPEED_A_D_recv.data[3]))
       {
        // Une os valores dos bytes de dados 4 e 5 do frame para gerar o valor do RPM
        rpmLidoD = canMsgReceive.data[5]*256 + canMsgReceive.data[4];
      } 
   }
}

void Throttle_CAN(int rpm_E, int rpm_D){

  acelerador_E = rpm_E*100.0/maxspeed;
  acelerador_D = rpm_D*100.0/maxspeed;
  
  if((acelerador_E > 0)&&(acelerador_E <= 100)){
    VCL_ThrottleE.data[4] = converterLSB(map(acelerador_E, 0, 100, 0, 32767));
    VCL_ThrottleE.data[5] = converterMSB(map(acelerador_E, 0, 100, 0, 32767));
  }
  
  if((acelerador_E < 0)&&(acelerador_E >= -100)){
    VCL_ThrottleE.data[4] = converterLSB(map(acelerador_E, -100, 0, 32768, 65535));
    VCL_ThrottleE.data[5] = converterMSB(map(acelerador_E, -100, 0, 32768, 65535));
  }

  if(acelerador_E == 0){
    VCL_ThrottleE.data[4] = converterLSB(0);
    VCL_ThrottleE.data[5] = converterMSB(0);
  }
  
  if(acelerador_E > 100){
    acelerador_E = 100;
  }
  if(acelerador_E < -100){
    acelerador_E = -100;
  }

  if((acelerador_D > 0)&&(acelerador_D <= 100)){
    VCL_ThrottleD.data[4] = converterLSB(map(acelerador_D, 0, 100, 0, 32767));
    VCL_ThrottleD.data[5] = converterMSB(map(acelerador_D, 0, 100, 0, 32767));
  }
  
  if((acelerador_D < 0)&&(acelerador_D >= -100)){
    VCL_ThrottleD.data[4] = converterLSB(map(acelerador_D, -100, 0, 32768, 65535));
    VCL_ThrottleD.data[5] = converterMSB(map(acelerador_D, -100, 0, 32768, 65535));
  }

  if(acelerador_D == 0){
    VCL_ThrottleD.data[4] = converterLSB(0);
    VCL_ThrottleD.data[5] = converterMSB(0);
  }
  
  if(acelerador_D > 100){
    acelerador_D = 100;
  }
  if(acelerador_D < -100){
    acelerador_D = -100;
  }
  
  mcp2515.sendMessage(&VCL_ThrottleE);
  mcp2515.sendMessage(&VCL_ThrottleD);
  
}

// Separa o byte mais significativo do valor a ser enviado 

byte converterMSB(long valor){
    byte byte1 = 0;
    while(valor >= 256){
      byte1++;
      valor -= 256;
    }
  return byte1;
}

// Separa o byte menos significativo do valor a ser enviado 

byte converterLSB(long valor){
   while(valor >= 256){
      valor -= 256;
    }
  return valor;
}

void mensagensCAN(){
  
  VCL_ThrottleE.can_id  = 0x627; // ID do driver esquerdo
  VCL_ThrottleE.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
  VCL_ThrottleE.data[0] = 0x2B;  // Tipo de mensagem (40h recebe, 2Bh envia)
  VCL_ThrottleE.data[1] = 0x18;  // Segundo byte do indice
  VCL_ThrottleE.data[2] = 0x32;  // Primeiro byte do indice
  VCL_ThrottleE.data[3] = 0x00;  // Sub-indice
  VCL_ThrottleE.data[4] = 0x00;  // Segundo byte de dados
  VCL_ThrottleE.data[5] = 0x00;  // Primeiro byte de dados
  VCL_ThrottleE.data[6] = 0x00;  // Byte de dado nao usado
  VCL_ThrottleE.data[7] = 0x00;  // Byte de dado nao usado

  VCL_ThrottleD.can_id  = 0x626; // ID do driver direito
  VCL_ThrottleD.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
  VCL_ThrottleD.data[0] = 0x2B;  // Tipo de mensagem (40h recebe, 2Bh envia)
  VCL_ThrottleD.data[1] = 0x18;  // Segundo byte do indice
  VCL_ThrottleD.data[2] = 0x32;  // Primeiro byte do indice
  VCL_ThrottleD.data[3] = 0x00;  // Sub-indice
  VCL_ThrottleD.data[4] = 0x00;  // Segundo byte de dados
  VCL_ThrottleD.data[5] = 0x00;  // Primeiro byte de dados
  VCL_ThrottleD.data[6] = 0x00;  // Byte de dado nao usado
  VCL_ThrottleD.data[7] = 0x00;  // Byte de dado nao usado
  
  MOTOR_SPEED_A_E.can_id  = 0x627; // ID do driver esquerdo
  MOTOR_SPEED_A_E.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
  MOTOR_SPEED_A_E.data[0] = 0x40;  // Tipo de mensagem (40h recebe, 2Bh envia)
  MOTOR_SPEED_A_E.data[1] = 0x07;  // Segundo byte do indice
  MOTOR_SPEED_A_E.data[2] = 0x32;  // Primeiro byte do indice
  MOTOR_SPEED_A_E.data[3] = 0x00;  // Sub-indice
  MOTOR_SPEED_A_E.data[4] = 0x00;  // Segundo byte de dados
  MOTOR_SPEED_A_E.data[5] = 0x00;  // Primeiro byte de dados
  MOTOR_SPEED_A_E.data[6] = 0x00;  // Byte de dado nao usado
  MOTOR_SPEED_A_E.data[7] = 0x00;  // Byte de dado nao usado
  
  MOTOR_SPEED_A_D.can_id  = 0x626; // ID do driver direito
  MOTOR_SPEED_A_D.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
  MOTOR_SPEED_A_D.data[0] = 0x40;  // Tipo de mensagem (40h recebe, 2Bh envia)
  MOTOR_SPEED_A_D.data[1] = 0x07;  // Segundo byte do indice
  MOTOR_SPEED_A_D.data[2] = 0x32;  // Primeiro byte do indice
  MOTOR_SPEED_A_D.data[3] = 0x00;  // Sub-indice
  MOTOR_SPEED_A_D.data[4] = 0x00;  // Segundo byte de dados
  MOTOR_SPEED_A_D.data[5] = 0x00;  // Primeiro byte de dados
  MOTOR_SPEED_A_D.data[6] = 0x00;  // Byte de dado nao usado
  MOTOR_SPEED_A_D.data[7] = 0x00;  // Byte de dado nao usado
  
  MOTOR_SPEED_A_E_recv.can_id  = 0x5A7; // ID do driver esquerdo
  MOTOR_SPEED_A_E_recv.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
  MOTOR_SPEED_A_E_recv.data[0] = 0x42;  // Tipo de mensagem (40h recebe, 2Bh envia)
  MOTOR_SPEED_A_E_recv.data[1] = 0x07;  // Segundo byte do indice
  MOTOR_SPEED_A_E_recv.data[2] = 0x32;  // Primeiro byte do indice
  MOTOR_SPEED_A_E_recv.data[3] = 0x00;  // Sub-indice
  
  MOTOR_SPEED_A_D_recv.can_id  = 0x5A6; // ID do driver direito
  MOTOR_SPEED_A_D_recv.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
  MOTOR_SPEED_A_D_recv.data[0] = 0x42;  // Tipo de mensagem (40h recebe, 2Bh envia)
  MOTOR_SPEED_A_D_recv.data[1] = 0x07;  // Segundo byte do indice
  MOTOR_SPEED_A_D_recv.data[2] = 0x32;  // Primeiro byte do indice
  MOTOR_SPEED_A_D_recv.data[3] = 0x00;  // Sub-indice
  
}

