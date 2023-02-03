#include <SPI.h>
#include <mcp2515.h>

struct can_frame VCL_ThrottleE; // acelerador_E Motor Esquerdo
struct can_frame VCL_ThrottleD; // acelerador_E Motor Direito
struct can_frame MAX_SPEED_E;   // Max Speed Motor Esquerdo
struct can_frame MAX_SPEED_D;   // Max Speed Motor Direito

MCP2515 mcp2515(10);

long valor = 0;

long t = 0;

int ENC_E_A = 2; // Canal A do encoder esquerdo
int ENC_E_B = 4; // Canal B do encoder esquerdo
int ENC_D_A = 3; // Canal A do encoder direito
int ENC_D_B = 5; // Canal B do encoder direito

long contE = 0;
long contD = 0;

long contAntE = 0;
long contAntD = 0;

int flagE = 0;
int flagD = 0;

int maxspeed = 2500;

int acelerador_E = 10;
int acelerador_D = 10;

int rpm_E = 760;
int rpm_D = 760;
int rpmLidoE = 0;
int rpmLidoD = 0;

void setup() {
  
  while (!Serial);
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();

  //attachInterrupt(digitalPinToInterrupt(ENC_E_A), LerEncoderE, RISING);
  //attachInterrupt(digitalPinToInterrupt(ENC_D_A), LerEncoderD, RISING);
  pinMode(ENC_E_B, INPUT_PULLUP);
  pinMode(ENC_D_B, INPUT_PULLUP);
  pinMode(ENC_E_A, INPUT_PULLUP);
  pinMode(ENC_D_A, INPUT_PULLUP);

  VCL_ThrottleE.can_id  = 0x627;
  VCL_ThrottleE.can_dlc = 8;
  VCL_ThrottleE.data[0] = 0x2B;
  VCL_ThrottleE.data[1] = 0x18;
  VCL_ThrottleE.data[2] = 0x32;
  VCL_ThrottleE.data[3] = 0x00;
  VCL_ThrottleE.data[4] = 0x00;
  VCL_ThrottleE.data[5] = 0x00;
  VCL_ThrottleE.data[6] = 0x00;
  VCL_ThrottleE.data[7] = 0x00;

  VCL_ThrottleD.can_id  = 0x626;
  VCL_ThrottleD.can_dlc = 8;
  VCL_ThrottleD.data[0] = 0x2B;
  VCL_ThrottleD.data[1] = 0x18;
  VCL_ThrottleD.data[2] = 0x32;
  VCL_ThrottleD.data[3] = 0x00;
  VCL_ThrottleD.data[4] = 0x00;
  VCL_ThrottleD.data[5] = 0x00;
  VCL_ThrottleD.data[6] = 0x00;
  VCL_ThrottleD.data[7] = 0x00;

  MAX_SPEED_E.can_id  = 0x627;
  MAX_SPEED_E.can_dlc = 8;
  MAX_SPEED_E.data[0] = 0x2B;
  MAX_SPEED_E.data[1] = 0x11;
  MAX_SPEED_E.data[2] = 0x30;
  MAX_SPEED_E.data[3] = 0x00;
  MAX_SPEED_E.data[4] = 0x00;
  MAX_SPEED_E.data[5] = 0x00;
  MAX_SPEED_E.data[6] = 0x00;
  MAX_SPEED_E.data[7] = 0x00;

  MAX_SPEED_D.can_id  = 0x626;
  MAX_SPEED_D.can_dlc = 8;
  MAX_SPEED_D.data[0] = 0x2B;
  MAX_SPEED_D.data[1] = 0x11;
  MAX_SPEED_D.data[2] = 0x30;
  MAX_SPEED_D.data[3] = 0x00;
  MAX_SPEED_D.data[4] = 0x00;
  MAX_SPEED_D.data[5] = 0x00;
  MAX_SPEED_D.data[6] = 0x00;
  MAX_SPEED_D.data[7] = 0x00;
  
}

void loop() {

  acelerador_E = rpm_E*100.0/maxspeed;
  acelerador_D = rpm_D*100.0/maxspeed;
  

  if(millis() - t > 100){
    mcp2515.sendMessage(&VCL_ThrottleE);
    mcp2515.sendMessage(&MAX_SPEED_E);
    mcp2515.sendMessage(&VCL_ThrottleD);
    mcp2515.sendMessage(&MAX_SPEED_D);

/*
    Serial.print(contE);
    Serial.print('\t');
    Serial.println(contD);
  */
    rpmLidoE = (contE - contAntE)*600/32;
    rpmLidoD = (contD - contAntD)*600/32;
    contAntE = contE;
    contAntD = contD;
    Serial.print(rpmLidoE);
    Serial.print('\t');
    Serial.println(rpmLidoD);
    
    t = millis();
    
  }
    
  if((digitalRead(ENC_D_A) == HIGH)&&(flagD == 0)){
    flagD = 1;
    if (digitalRead(ENC_D_B) == LOW) {
      contD++;
    }
    else {
      contD--;
    }
  }
  
  if(digitalRead(ENC_D_A) == LOW){
    flagD = 0;
  }




  if((digitalRead(ENC_E_A) == HIGH)&&(flagE == 0)){
    flagE = 1;
    if (digitalRead(ENC_E_B) == HIGH) {
      contE++;
    }
    else {
      contE--;
    }
  }
  
  if(digitalRead(ENC_E_A) == LOW){
    flagE = 0;
  }
  
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

void LerEncoderE() {
  if (digitalRead(ENC_E_B) == HIGH) {
    contE++;
  }
  else {
    contE--;
  }
}

void LerEncoderD() {
  if (digitalRead(ENC_D_B) == LOW) {
    contD++;
  }
  else {
    contD--;
  }
}

