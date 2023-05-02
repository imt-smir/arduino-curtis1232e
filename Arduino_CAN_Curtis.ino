//
// Este código consiste no controle do RPM desejado dos 2 motores,
// e na leitura em tempo real dessas velocidades pelo monitor serial
// 
// Este código foi pensado para uso de um Arduino Uno em conjunto com um módulo CAN MCP2515
//

#include <SPI.h>
#include <mcp2515.h>

// Mensagens CAN

struct can_frame VCL_ThrottleE; // acelerador_E Motor Esquerdo
struct can_frame VCL_ThrottleD; // acelerador_D Motor Direito
struct can_frame MOTOR_SPEED_A_E; // Leitura RPM Esquerdo (Solicitação)
struct can_frame MOTOR_SPEED_A_D; // Leitura RPM Direito  (Solicitação)
struct can_frame MOTOR_SPEED_A_E_recv; // Leitura RPM Esquerdo (Recebimento)
struct can_frame MOTOR_SPEED_A_D_recv; // Leitura RPM Direito  (Recebimento)
struct can_frame canMsgReceive; // Receber mensagem CAN

MCP2515 mcp2515(10); // Pino CS do Módulo CAN

long previousMillis = 0;

int maxspeed = 760; // Velocidade máxima do motor definida nos parâmetros do driver (RPM)

int acelerador_E = 0; // Porcentagem da velocidade aplicada no motor esq. em relação a velocidade máxima
int acelerador_D = 0; // Porcentagem da velocidade aplicada no motor dir. em relação a velocidade máxima

int rpm_E = 300; // RPM desejado no motor esquerdo
int rpm_D = 300; // RPM desejado no motor direito

int rpmLidoE = 0; // RPM lido no motor esquerdo
int rpmLidoD = 0; // RPM lido no motor direito

int periodoLeitura = 120; // tempo de leitura em ms

int tarefa_atual = 0;
int n_tarefas = 3;

void setup() {
  
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); // BaudRate do CAN do driver
  mcp2515.setNormalMode();

  mensagensCAN(); // Declara o frame das mensagens CAN
  
}

void loop() {

  if(millis() - previousMillis > periodoLeitura/n_tarefas){

    if(tarefa_atual == 0){
      Throttle_CAN(rpm_E, rpm_D); // Envia a velocidade desejada de cada motor em RPM
      tarefa_atual++;
    }

    else if(tarefa_atual == 1){
      mcp2515.sendMessage(&MOTOR_SPEED_A_E); // Envia a solicitação de leitura da velocidade do motor esq.
      Serial.print(rpmLidoE);
      Serial.print('\t');
      tarefa_atual++;
    }
    
    else if(tarefa_atual == 2){
      mcp2515.sendMessage(&MOTOR_SPEED_A_D); // Envia a solicitação de leitura da velocidade do motor dir.
      Serial.println(rpmLidoD);
      tarefa_atual = 0;
    }
    
    previousMillis = millis();
  }

  LerRPM();
   
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
  VCL_ThrottleE.data[6] = 0x00;  // Byte de dados não usado
  VCL_ThrottleE.data[7] = 0x00;  // Byte de dados não usado

  VCL_ThrottleD.can_id  = 0x626; // ID do driver direito
  VCL_ThrottleD.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
  VCL_ThrottleD.data[0] = 0x2B;  // Tipo de mensagem (40h recebe, 2Bh envia)
  VCL_ThrottleD.data[1] = 0x18;  // Segundo byte do indice
  VCL_ThrottleD.data[2] = 0x32;  // Primeiro byte do indice
  VCL_ThrottleD.data[3] = 0x00;  // Sub-indice
  VCL_ThrottleD.data[4] = 0x00;  // Segundo byte de dados
  VCL_ThrottleD.data[5] = 0x00;  // Primeiro byte de dados
  VCL_ThrottleD.data[6] = 0x00;  // Byte de dados não usado
  VCL_ThrottleD.data[7] = 0x00;  // Byte de dados não usado
  
  MOTOR_SPEED_A_E.can_id  = 0x627; // ID do driver esquerdo
  MOTOR_SPEED_A_E.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
  MOTOR_SPEED_A_E.data[0] = 0x40;  // Tipo de mensagem (40h recebe, 2Bh envia)
  MOTOR_SPEED_A_E.data[1] = 0x07;  // Segundo byte do indice
  MOTOR_SPEED_A_E.data[2] = 0x32;  // Primeiro byte do indice
  MOTOR_SPEED_A_E.data[3] = 0x00;  // Sub-indice
  MOTOR_SPEED_A_E.data[4] = 0x00;  // Segundo byte de dados
  MOTOR_SPEED_A_E.data[5] = 0x80;  // Primeiro byte de dados
  MOTOR_SPEED_A_E.data[6] = 0x00;  // Byte de dados não usado
  MOTOR_SPEED_A_E.data[7] = 0x00;  // Byte de dados não usado
  
  MOTOR_SPEED_A_D.can_id  = 0x626; // ID do driver direito
  MOTOR_SPEED_A_D.can_dlc = 8;     // Quantidade de bytes da mensagem CAN
  MOTOR_SPEED_A_D.data[0] = 0x40;  // Tipo de mensagem (40h recebe, 2Bh envia)
  MOTOR_SPEED_A_D.data[1] = 0x07;  // Segundo byte do indice
  MOTOR_SPEED_A_D.data[2] = 0x32;  // Primeiro byte do indice
  MOTOR_SPEED_A_D.data[3] = 0x00;  // Sub-indice
  MOTOR_SPEED_A_D.data[4] = 0x00;  // Segundo byte de dados
  MOTOR_SPEED_A_D.data[5] = 0x00;  // Primeiro byte de dados
  MOTOR_SPEED_A_D.data[6] = 0x00;  // Byte de dados não usado
  MOTOR_SPEED_A_D.data[7] = 0x00;  // Byte de dados não usado
  
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
