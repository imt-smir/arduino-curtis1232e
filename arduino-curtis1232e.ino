#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsgSend;
struct can_frame canMsgSend2;
struct can_frame canMsgSend3;
struct can_frame canMsgReceive;
MCP2515 mcp2515(10);
 


void setup() {
  canMsgSend.can_id  = 0x627; // ID CANbus
  canMsgSend.can_dlc = 8;     // Tamanho da mensagem em bytes
  canMsgSend.data[0] = 0x2B;  // Controle
  canMsgSend.data[1] = 0x18;  // Segundo byte do indice
  canMsgSend.data[2] = 0x32;  // Primeiro byte do indice
  canMsgSend.data[3] = 0x00;  // sub-indice
  canMsgSend.data[4] = 0xFF;  //
  canMsgSend.data[5] = 0x7F;  //
  canMsgSend.data[6] = 0x00;  //
  canMsgSend.data[7] = 0x00;  //

  canMsgSend2.can_id  = 0x627; // ID CANbus
  canMsgSend2.can_dlc = 8;     // Tamanho da mensagem em bytes
  canMsgSend2.data[0] = 0x2B;  // Controle
  canMsgSend2.data[1] = 0x19;  // Segundo byte do indice
  canMsgSend2.data[2] = 0x32;  // Primeiro byte do indice
  canMsgSend2.data[3] = 0x00;  // sub-indice
  canMsgSend2.data[4] = 0x00;  //
  canMsgSend2.data[5] = 0x00;  //
  canMsgSend2.data[6] = 0x00;  //
  canMsgSend2.data[7] = 0x00;  //

  canMsgSend3.can_id  = 0x627; // ID CANbus
  canMsgSend3.can_dlc = 8;     // Tamanho da mensagem em bytes
  canMsgSend3.data[0] = 0x40;  // Controle
  canMsgSend3.data[1] = 0x1A;  // Segundo byte do indice
  canMsgSend3.data[2] = 0x32;  // Primeiro byte do indice
  canMsgSend3.data[3] = 0x00;  // sub-indice
  canMsgSend3.data[4] = 0x00;  //
  canMsgSend3.data[5] = 0x00;  //
  canMsgSend3.data[6] = 0x00;  //
  canMsgSend3.data[7] = 0x00;  //

  mcp2515.sendMessage(&canMsgSend2);
  
  while (!Serial);
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS);
  mcp2515.setNormalMode();
  
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}

void loop() {
  mcp2515.sendMessage(&canMsgSend);
  //mcp2515.sendMessage(&canMsgSend3);

  Serial.println("Messages sent");

   if (mcp2515.readMessage(&canMsgReceive) == MCP2515::ERROR_OK) {
    Serial.print(canMsgReceive.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsgReceive.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<canMsgReceive.can_dlc; i++)  {  // print the data
      Serial.print(canMsgReceive.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();      
  }
  
  delay(100);
}
