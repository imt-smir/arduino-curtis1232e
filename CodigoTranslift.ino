#include "robolo.h"
#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(53);

vel_t teleop_linear = 0.05;
vel_t teleop_angular = 0.0;
vel_t motE_set_point, motD_set_point; // Set Points para as velocidades desejadas

long t = 0;

int tarefa_atual = 0;
int n_tarefas = 4;

int rpmLidoE = 0;
int rpmLidoD = 0;

int acelerador_E = 10;
int acelerador_D = 10;

int maxspeed = 760;
float transmissao = 60.0/(46*38); // relacao de transmissao 60.0/(46*38)

struct can_frame VCL_ThrottleE; // acelerador_E Motor Esquerdo
struct can_frame VCL_ThrottleD; // acelerador_E Motor Direito
struct can_frame MAX_SPEED_E;   // Max Speed Motor Esquerdo
struct can_frame MAX_SPEED_D;   // Max Speed Motor Direito
struct can_frame MOTOR_SPEED_A_E; // Leitura RPM Esquerdo
struct can_frame MOTOR_SPEED_A_D; // Leitura RPM Direito
struct can_frame MOTOR_SPEED_A_E_recv; // 
struct can_frame MOTOR_SPEED_A_D_recv; // 
struct can_frame canMsgReceive; // Receber mensagem CAN


/* Inicialização rosserial */
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> sub_teleop("cmd_vel", teleop_callback);
tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);


#include <std_msgs/Float32.h>
std_msgs::Float32 test1_msg;
ros::Publisher test1_pub("test1", &test1_msg);


/* Variáveis de odometria */
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
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Serial1.begin(9600);

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

  MAX_SPEED_E.can_id  = 0x627; // ID do driver esquerdo
  MAX_SPEED_E.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
  MAX_SPEED_E.data[0] = 0x2B;  // Tipo de mensagem (40h recebe, 2Bh envia)
  MAX_SPEED_E.data[1] = 0x11;  // Segundo byte do indice
  MAX_SPEED_E.data[2] = 0x30;  // Primeiro byte do indice
  MAX_SPEED_E.data[3] = 0x00;  // Sub-indice
  MAX_SPEED_E.data[4] = 0x00;  // Segundo byte de dados
  MAX_SPEED_E.data[5] = 0x00;  // Primeiro byte de dados
  MAX_SPEED_E.data[6] = 0x00;  // Byte de dado nao usado
  MAX_SPEED_E.data[7] = 0x00;  // Byte de dado nao usado

  MAX_SPEED_D.can_id  = 0x626; // ID do driver direito
  MAX_SPEED_D.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
  MAX_SPEED_D.data[0] = 0x2B;  // Tipo de mensagem (40h recebe, 2Bh envia)
  MAX_SPEED_D.data[1] = 0x11;  // Segundo byte do indice
  MAX_SPEED_D.data[2] = 0x30;  // Primeiro byte do indice
  MAX_SPEED_D.data[3] = 0x00;  // Sub-indice
  MAX_SPEED_D.data[4] = 0x00;  // Segundo byte de dados
  MAX_SPEED_D.data[5] = 0x00;  // Primeiro byte de dados
  MAX_SPEED_D.data[6] = 0x00;  // Byte de dado nao usado
  MAX_SPEED_D.data[7] = 0x00;  // Byte de dado nao usado

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

  nh.initNode();
  nh.subscribe(sub_teleop);
  broadcaster.init(nh);
  nh.advertise(test1_pub);
  nh.advertise(odom_pub);

  transforma_cinematica_robo(teleop_linear, teleop_angular, &motE_set_point, &motD_set_point); 
  Throttle_CAN(motE_set_point * 60.0/transmissao, motD_set_point * 60.0/transmissao); // rpm
  
}

void loop() {
  nh.spinOnce(); // Sintaxe para iniciar o loop do ROS

  if(millis() - t > 100){
    
    mcp2515.sendMessage(&MOTOR_SPEED_A_E);
    envia_odometria(rpmLidoE * transmissao/60.0, rpmLidoE * transmissao/60.0);
    t = millis();
    
  }

  
  LerRPM_E();
  LerRPM_D();

  
   
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
  //primeira mudança - PERIODO_LEITURA agora vai ser calculado a todo momento (millis() - t_anterior)
  //segunda mudança seguindo o codigo atualizado - transformar mot*_vel_atual de rot/s para m/s para isso multiplica por 2 * pi * RAIO_RODA
  mot1_vel_atual = mot1_vel_atual * two_pi * RAIO_RODA;
  mot2_vel_atual = mot2_vel_atual * two_pi * RAIO_RODA;
  /* Odometria */
  // Criando TF e publicando Odom

  dxy = (mot1_vel_atual + mot2_vel_atual) * 100 / (2 * 1000); // Distancia instantanea (modulo)
  dth = (-mot1_vel_atual + mot2_vel_atual) * 100 / (1000 * DISTANCIA_ENTRE_RODAS); // Rotação instantanea
  dx = cos(dth) * dxy;
  dy = sin(dth) * dxy;
  
  theta += dth;
  x_pos += (cos(theta) * dx - sin(theta) * dy);
  y_pos += (sin(theta) * dx + cos(theta) * dy);

//  if (theta >= two_pi) theta -= two_pi; //Comentado pelo joao
//  if (theta <= -two_pi) theta += two_pi;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);
  geometry_msgs::TransformStamped t;

  xy_pos += dxy; // NOVO

  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = x_pos;
  t.transform.translation.y = y_pos;
  t.transform.translation.z = 0.0;
  t.transform.rotation = odom_quat;
  t.header.stamp = nh.now(); //talvez

  broadcaster.sendTransform(t);

  odom_msg.header.stamp = nh.now(); //talvez
  odom_msg.header.frame_id = "odom";
  odom_msg.pose.pose.position.x = x_pos;
  odom_msg.pose.pose.position.y = y_pos;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;


  vx = (mot1_vel_atual + mot2_vel_atual) / 2;
  vth = (mot1_vel_atual - mot2_vel_atual) / (43.6/100.0);
  odom_msg.child_frame_id = base_link;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = 0.0;
  odom_msg.twist.twist.angular.z = vth;

  //Serial.println(xy_pos);
  Serial1.println(xy_pos);

  odom_pub.publish(&odom_msg);

  test1_msg.data = xy_pos;
  test1_pub.publish(&test1_msg);

}




void LerRPM_E(){
  if (mcp2515.readMessage(&canMsgReceive) == MCP2515::ERROR_OK) {
        
     if ((canMsgReceive.can_id == MOTOR_SPEED_A_E_recv.can_id)&&
       (canMsgReceive.can_dlc == MOTOR_SPEED_A_E_recv.can_dlc)&&
       (canMsgReceive.data[0] == MOTOR_SPEED_A_E_recv.data[0])&&
       (canMsgReceive.data[1] == MOTOR_SPEED_A_E_recv.data[1])&&
       (canMsgReceive.data[2] == MOTOR_SPEED_A_E_recv.data[2])&&
       (canMsgReceive.data[3] == MOTOR_SPEED_A_E_recv.data[3]))
       {
        rpmLidoE = canMsgReceive.data[5]*256 + canMsgReceive.data[4];
      }   
   }
}

void LerRPM_D(){
  if (mcp2515.readMessage(&canMsgReceive) == MCP2515::ERROR_OK) {
        
     if ((canMsgReceive.can_id == MOTOR_SPEED_A_D_recv.can_id)&&
       (canMsgReceive.can_dlc == MOTOR_SPEED_A_D_recv.can_dlc)&&
       (canMsgReceive.data[0] == MOTOR_SPEED_A_D_recv.data[0])&&
       (canMsgReceive.data[1] == MOTOR_SPEED_A_D_recv.data[1])&&
       (canMsgReceive.data[2] == MOTOR_SPEED_A_D_recv.data[2])&&
       (canMsgReceive.data[3] == MOTOR_SPEED_A_D_recv.data[3]))
       {
        rpmLidoD = canMsgReceive.data[5]*256 + canMsgReceive.data[4];
      }   
   }
}


void Throttle_CAN(int rpm_E, int rpm_D){

  acelerador_E = rpm_E*100.0/maxspeed;
  acelerador_D = rpm_D*100.0/maxspeed;
  
  if((acelerador_E > 0)&&(acelerador_E <= 100)){
    MAX_SPEED_E.data[4] = converterLSB(maxspeed);
    MAX_SPEED_E.data[5] = converterMSB(maxspeed);
    VCL_ThrottleE.data[4] = converterLSB(map(acelerador_E, 0, 100, 0, 32767));
    VCL_ThrottleE.data[5] = converterMSB(map(acelerador_E, 0, 100, 0, 32767));
  }
  
  if((acelerador_E < 0)&&(acelerador_E >= -100)){
    MAX_SPEED_E.data[4] = converterLSB(maxspeed);
    MAX_SPEED_E.data[5] = converterMSB(maxspeed);
    VCL_ThrottleE.data[4] = converterLSB(map(acelerador_E, -100, 0, 32768, 65535));
    VCL_ThrottleE.data[5] = converterMSB(map(acelerador_E, -100, 0, 32768, 65535));
  }

  if(acelerador_E == 0){
    MAX_SPEED_E.data[4] = converterLSB(100);
    MAX_SPEED_E.data[5] = converterMSB(100);
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
    MAX_SPEED_D.data[4] = converterLSB(maxspeed);
    MAX_SPEED_D.data[5] = converterMSB(maxspeed);
    VCL_ThrottleD.data[4] = converterLSB(map(acelerador_D, 0, 100, 0, 32767));
    VCL_ThrottleD.data[5] = converterMSB(map(acelerador_D, 0, 100, 0, 32767));
  }
  
  if((acelerador_D < 0)&&(acelerador_D >= -100)){
    MAX_SPEED_D.data[4] = converterLSB(maxspeed);
    MAX_SPEED_D.data[5] = converterMSB(maxspeed);
    VCL_ThrottleD.data[4] = converterLSB(map(acelerador_D, -100, 0, 32768, 65535));
    VCL_ThrottleD.data[5] = converterMSB(map(acelerador_D, -100, 0, 32768, 65535));
  }

  if(acelerador_D == 0){
    MAX_SPEED_D.data[4] = converterLSB(100);
    MAX_SPEED_D.data[5] = converterMSB(100);
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
  //mcp2515.sendMessage(&MAX_SPEED_E);
  mcp2515.sendMessage(&VCL_ThrottleD);
  //mcp2515.sendMessage(&MAX_SPEED_D);
}

byte converterMSB(long valor){
    byte byte1 = 0;
    while(valor >= 256){
      byte1++;
      valor -= 256;
    }
  return byte1;
}

byte converterLSB(long valor){
   while(valor >= 256){
      valor -= 256;
    }
  return valor;
}

