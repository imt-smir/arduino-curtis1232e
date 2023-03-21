/* This example puts all received datas in the file 'logfile.txt',
 * you can compare them with the transmited datas to test whether they are
 * right
 */
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include "can4linux.h"
#include <math.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "translift.h"


vel_t motE_set_point, motD_set_point; // Set Points para as velocidades desejadas

long t = 0;

int tarefa_atual = 0;
int n_tarefas = 4;

short rpmLidoE = 0;
short rpmLidoD = 0;

int maxspeed = 760;
float transmissao = 60.0 / (46 * 38); // relacao de transmissao 60.0/(46*38)
int acelerador_E = 0;
int acelerador_D = 0;

#define STDDEV "can1"
#define MAX 2

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

int set_bitrate(
    int can_fd, /* device descriptor */
    int baud    /* bit rate */
)

{
   Config_par_t cfg;
   volatile Command_par_t cmd;

   cmd.cmd = CMD_STOP;
   ioctl(can_fd, CAN_IOCTL_COMMAND, &cmd);

   cfg.target = CONF_TIMING;
   cfg.val1 = baud;
   ioctl(can_fd, CAN_IOCTL_CONFIG, &cfg);

   cmd.cmd = CMD_START;
   ioctl(can_fd, CAN_IOCTL_COMMAND, &cmd);
   return 0;
}

long long millis()
{
   struct timeval tv;

   gettimeofday(&tv, NULL);

   return (((long long)tv.tv_sec) * 1000) + (tv.tv_usec / 1000);
}

canmsg_t MOTOR_SPEED_A_E[1];
canmsg_t MOTOR_SPEED_A_D[1];
canmsg_t VCL_ThrottleE[1];
canmsg_t VCL_ThrottleD[1];
canmsg_t MOTOR_SPEED_A_E_recv[1];
canmsg_t MOTOR_SPEED_A_D_recv[1];

void Throttle_CAN(int rpm_E, int rpm_D);
int converterMSB(long valor);
int converterLSB(long valor);
void transforma_cinematica_robo(vel_t linear, vel_t angular, vel_t *mot1, vel_t *mot2);
int map(float xi, float xmin, float xmax, float ymin, float ymax);
void teleop_callback(const geometry_msgs::Twist& vel);


//



/***********************************************************************
 *
 * main
 *
 */

int main(int argc, char **argv)
{
   ros::init(argc, argv, "odometry");
   ros::NodeHandle nh;
   tf::TransformBroadcaster broadcaster;
   nav_msgs::Odometry odom_msg;
   ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50, &odom_msg);
   ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, teleop_callback);
   ros::Publisher teste_pub = nh.advertise<std_msgs::String>("teste", 50);

   vel_t mot1_vel_atual;
   vel_t mot2_vel_atual;
   
   int fd;
   int got;
   int c, count = 0;
   int sent;
   long long numero = 0;
   long long t_anterior = millis();
   char *pname;
   extern char *optarg;
   extern int optind;
   int ntarefas = 3;
   int tarefa_atual = 0;
   int periodo_de_leitura = 100;

   FILE *logFile;
   canmsg_t rx[1];
   char device[50];

   MOTOR_SPEED_A_E[0].id = 0x627;
   MOTOR_SPEED_A_E[0].flags = 0;
   MOTOR_SPEED_A_E[0].length = 8;
   MOTOR_SPEED_A_E[0].data[0] = 0x40;
   MOTOR_SPEED_A_E[0].data[1] = 0x07;
   MOTOR_SPEED_A_E[0].data[2] = 0x32;
   MOTOR_SPEED_A_E[0].data[3] = 0x00;
   MOTOR_SPEED_A_E[0].data[4] = 0x00;
   MOTOR_SPEED_A_E[0].data[5] = 0x00;
   MOTOR_SPEED_A_E[0].data[6] = 0x00;
   MOTOR_SPEED_A_E[0].data[7] = 0x00;

   MOTOR_SPEED_A_D[0].id = 0x626;
   MOTOR_SPEED_A_D[0].flags = 0;
   MOTOR_SPEED_A_D[0].length = 8;
   MOTOR_SPEED_A_D[0].data[0] = 0x40;
   MOTOR_SPEED_A_D[0].data[1] = 0x07;
   MOTOR_SPEED_A_D[0].data[2] = 0x32;
   MOTOR_SPEED_A_D[0].data[3] = 0x00;
   MOTOR_SPEED_A_D[0].data[4] = 0x00;
   MOTOR_SPEED_A_D[0].data[5] = 0x00;
   MOTOR_SPEED_A_D[0].data[6] = 0x00;
   MOTOR_SPEED_A_D[0].data[7] = 0x00;

   VCL_ThrottleE[0].id = 0x627;
   VCL_ThrottleE[0].flags = 0;
   VCL_ThrottleE[0].length = 8;
   VCL_ThrottleE[0].data[0] = 0x2B;
   VCL_ThrottleE[0].data[1] = 0x18;
   VCL_ThrottleE[0].data[2] = 0x32;
   VCL_ThrottleE[0].data[3] = 0x00;
   VCL_ThrottleE[0].data[4] = 0x00;
   VCL_ThrottleE[0].data[5] = 0x40;
   VCL_ThrottleE[0].data[6] = 0x00;
   VCL_ThrottleE[0].data[7] = 0x00;

   VCL_ThrottleD[0].id = 0x626;
   VCL_ThrottleD[0].flags = 0;
   VCL_ThrottleD[0].length = 8;
   VCL_ThrottleD[0].data[0] = 0x2B;
   VCL_ThrottleD[0].data[1] = 0x18;
   VCL_ThrottleD[0].data[2] = 0x32;
   VCL_ThrottleD[0].data[3] = 0x00;
   VCL_ThrottleD[0].data[4] = 0x00;
   VCL_ThrottleD[0].data[5] = 0x40;
   VCL_ThrottleD[0].data[6] = 0x00;
   VCL_ThrottleD[0].data[7] = 0x00;

   MOTOR_SPEED_A_E_recv[0].id = 0x5A7;
   MOTOR_SPEED_A_E_recv[0].flags = 0;
   MOTOR_SPEED_A_E_recv[0].length = 8;
   MOTOR_SPEED_A_E_recv[0].data[0] = 0x42;
   MOTOR_SPEED_A_E_recv[0].data[1] = 0x07;
   MOTOR_SPEED_A_E_recv[0].data[2] = 0x32;
   MOTOR_SPEED_A_E_recv[0].data[3] = 0x00;

   MOTOR_SPEED_A_D_recv[0].id = 0x5A6;
   MOTOR_SPEED_A_D_recv[0].flags = 0;
   MOTOR_SPEED_A_D_recv[0].length = 8;
   MOTOR_SPEED_A_D_recv[0].data[0] = 0x42;
   MOTOR_SPEED_A_D_recv[0].data[1] = 0x07;
   MOTOR_SPEED_A_D_recv[0].data[2] = 0x32;
   MOTOR_SPEED_A_D_recv[0].data[3] = 0x00;

   printf("usage: %s [dev] \n", argv[0]);
   if (argc > 1)
   {
      sprintf(device, "/dev/%s", argv[1]);
   }
   else
   {
      sprintf(device, "/dev/%s", STDDEV);
   }

   fd = open(device, O_RDWR | O_NONBLOCK);
   if (fd < 0)
   {
      fprintf(stderr, "Error opening CAN device %s\n", device);
      perror("open");
      exit(1);
   }

   set_bitrate(fd, 500);

   printf("using CAN device %s\n", device);

   ros::Rate r(10);
   while (ros::ok())
   {
   
      got = read(fd, rx, 50);
      
      if ((rx[0].id == MOTOR_SPEED_A_E_recv[0].id) &&
          (rx[0].flags == MOTOR_SPEED_A_E_recv[0].flags) &&
          (rx[0].length == MOTOR_SPEED_A_E_recv[0].length) &&
          (rx[0].data[0] == MOTOR_SPEED_A_E_recv[0].data[0]) &&
          (rx[0].data[1] == MOTOR_SPEED_A_E_recv[0].data[1]) &&
          (rx[0].data[2] == MOTOR_SPEED_A_E_recv[0].data[2]) &&
          (rx[0].data[3] == MOTOR_SPEED_A_E_recv[0].data[3]))
      {
         rpmLidoE = (rx[0].data[5] * 256 + rx[0].data[4]);
         printf("%d\n", rpmLidoE);
      }

      if ((rx[0].id == MOTOR_SPEED_A_D_recv[0].id) &&
          (rx[0].flags == MOTOR_SPEED_A_D_recv[0].flags) &&
          (rx[0].length == MOTOR_SPEED_A_D_recv[0].length) &&
          (rx[0].data[0] == MOTOR_SPEED_A_D_recv[0].data[0]) &&
          (rx[0].data[1] == MOTOR_SPEED_A_D_recv[0].data[1]) &&
          (rx[0].data[2] == MOTOR_SPEED_A_D_recv[0].data[2]) &&
          (rx[0].data[3] == MOTOR_SPEED_A_D_recv[0].data[3]))
      {
         rpmLidoD = (rx[0].data[5] * 256 + rx[0].data[4]);
      }

      if ((millis() - t_anterior) > periodo_de_leitura / ntarefas)
      {

         if (tarefa_atual == 0)
         {
            transforma_cinematica_robo(teleop_linear, teleop_angular, &motE_set_point, &motD_set_point);
            Throttle_CAN(motE_set_point * 60.0 / transmissao, motD_set_point * 60.0 / transmissao); // Envia a velocidade desejada de cada motor em RPM
            sent = write(fd, VCL_ThrottleE, 1);
            sent = write(fd, VCL_ThrottleD, 1);
            //printf("%f\n", teleop_linear);

            tarefa_atual++;
         }

         else if (tarefa_atual == 1)
         {
            sent = write(fd, MOTOR_SPEED_A_E, 1); // Envia a solicitação de leitura da velocidade do motor esq.
            tarefa_atual++;
         }

         else if (tarefa_atual == 2)
         {
            sent = write(fd, MOTOR_SPEED_A_D, 1); // Envia a solicitação de leitura da velocidade do motor dir.
            tarefa_atual++;
         }

         else
         {
            // primeira mudança - PERIODO_LEITURA agora vai ser calculado a todo momento (millis() - t_anterior)
            // segunda mudança seguindo o codigo atualizado - transformar mot*_vel_atual de rot/s para m/s para isso multiplica por 2 * pi * RAIO_RODA
            mot1_vel_atual = rpmLidoE * transmissao / 60.0 * two_pi * RAIO_RODA;
            mot2_vel_atual = rpmLidoD * transmissao / 60.0 * two_pi * RAIO_RODA;
            /* Odometria */
            // Criando TF e publicando Odom

            dxy = (mot1_vel_atual + mot2_vel_atual) * 4 *(millis() - t_anterior) / (2 * 1000);                      // Distancia instantanea (modulo)
            dth = (-mot1_vel_atual + mot2_vel_atual) * 4 * (millis() - t_anterior) / (1000 * DISTANCIA_ENTRE_RODAS); // Rotação instantanea
            dx = cos(dth) * dxy;
            dy = sin(dth) * dxy;

            theta += dth;
            x_pos += (cos(theta) * dx - sin(theta) * dy);
            y_pos += (sin(theta) * dx + cos(theta) * dy);

            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
            geometry_msgs::TransformStamped t;
            xy_pos += dxy; // NOVO

            t.header.frame_id = "odom";
            t.child_frame_id = "base_link";
            t.transform.translation.x = x_pos;
            t.transform.translation.y = y_pos;
            t.transform.translation.z = 0.0;
            t.transform.rotation = odom_quat;
            t.header.stamp = ros::Time::now(); // talvez

            broadcaster.sendTransform(t);

            odom_msg.header.stamp = ros::Time::now(); // talvez
            odom_msg.header.frame_id = "odom";
            odom_msg.pose.pose.position.x = x_pos;
            odom_msg.pose.pose.position.y = y_pos;
            odom_msg.pose.pose.position.z = 0.0;
            odom_msg.pose.pose.orientation = odom_quat;

            vx = (mot1_vel_atual + mot2_vel_atual) / 2;
            vth = (mot1_vel_atual - mot2_vel_atual) / (43.6 / 100.0);
            odom_msg.child_frame_id = base_link;
            odom_msg.twist.twist.linear.x = vx;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.angular.z = vth;

            odom_pub.publish(odom_msg);

            std_msgs::String msg;

	    std::stringstream ss;
	    ss << "rpm esquerdo " << rpmLidoE << "\t Rpm direito" << rpmLidoD;
	    msg.data = ss.str();
	    /**
	     * The publish() function is how you send messages. The parameter
	     * is the message object. The type of this object must agree with the type
	     * given as a template parameter to the advertise<>() call, as was done
	     * in the constructor above.
	     */
		// %Tag(PUBLISH)%
	    teste_pub.publish(msg);
            

            tarefa_atual = 0;


         }

         ++count;

         // printf("%lld\n", millis()-t_anterior);
         // printf("%d\n", velE);
         t_anterior = millis();

         
      }
      ros::spinOnce();
      
   }
   printf("count: %d\n", count);
   close(fd);

   

   return 0;
}

void Throttle_CAN(int rpm_E, int rpm_D)
{
   acelerador_E = rpm_E * 100.0 / maxspeed;
   acelerador_D = rpm_D * 100.0 / maxspeed;

   if ((acelerador_E > 0) && (acelerador_E <= 100))
   {
      VCL_ThrottleE[0].data[4] = converterLSB(map(acelerador_E, 0, 100, 0, 32767));
      VCL_ThrottleE[0].data[5] = converterMSB(map(acelerador_E, 0, 100, 0, 32767));
   }

   if ((acelerador_E < 0) && (acelerador_E >= -100))
   {
      VCL_ThrottleE[0].data[4] = converterLSB(map(acelerador_E, -100, 0, 32768, 65535));
      VCL_ThrottleE[0].data[5] = converterMSB(map(acelerador_E, -100, 0, 32768, 65535));
   }

   if (acelerador_E == 0)
   {
      VCL_ThrottleE[0].data[4] = converterLSB(0);
      VCL_ThrottleE[0].data[5] = converterMSB(0);
   }

   if (acelerador_E > 100)
      acelerador_E == 100;

   if (acelerador_E < -100)
      acelerador_E == -100;

   if ((acelerador_D > 0) && (acelerador_D <= 100))
   {
      VCL_ThrottleD[0].data[4] = converterLSB(map(acelerador_D, 0, 100, 0, 32767));
      VCL_ThrottleD[0].data[5] = converterMSB(map(acelerador_D, 0, 100, 0, 32767));
   }

   if ((acelerador_D < 0) && (acelerador_D >= -100))
   {
      VCL_ThrottleD[0].data[4] = converterLSB(map(acelerador_D, -100, 0, 32768, 65535));
      VCL_ThrottleD[0].data[5] = converterMSB(map(acelerador_D, -100, 0, 32768, 65535));
   }

   if (acelerador_D == 0)
   {
      VCL_ThrottleD[0].data[4] = converterLSB(0);
      VCL_ThrottleD[0].data[5] = converterMSB(0);
   }

   if (acelerador_D > 100)
      acelerador_D == 100;

   if (acelerador_D < -100)
      acelerador_D == -100;
}

int converterMSB(long valor)
{
   int bytel = 0;
   while (valor >= 256)
   {
      valor -= 256;
      bytel++;
   }
   return (bytel);
}

int converterLSB(long valor)
{
   while (valor >= 256)
      valor -= 256;
   return valor;
}

int map(float xi, float xmin, float xmax, float ymin, float ymax)
{
   return ((xi - xmin) * (ymax - ymin) / (xmax - xmin) + ymin);
}

void transforma_cinematica_robo(vel_t linear, vel_t angular, vel_t *mot1, vel_t *mot2)
{
   *mot1 = (2 * linear - angular * DISTANCIA_ENTRE_RODAS) * RADIANOS_PARA_ROT / (2 * RAIO_RODA);
   *mot2 = (2 * linear + angular * DISTANCIA_ENTRE_RODAS) * RADIANOS_PARA_ROT / (2 * RAIO_RODA);
   // mot1 mot2 está em rotações por segundo
}

void teleop_callback(const geometry_msgs::Twist& vel)
{ // Função de callback que recebe do tópico "cmd_vel" a velocidade desejada pelo usuário
   teleop_linear = vel.linear.x;
   teleop_angular = vel.angular.z;
   //ROS_INFO("I heard: [%f][%f]", teleop_linear, teleop_angular);
   //printf("%f\n", teleop_linear);
}
