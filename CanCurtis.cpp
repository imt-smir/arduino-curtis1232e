#include "CanCurtis.h"
#include <mcp2515.h>
 
Curtis::Curtis(int ID)
{ 
   struct can_frame canMsgSend;
   MCP2515 mcp2515(10);
   int id = ID + 0x600;
}
 
void Curtis::Throttle()
{
   canMsgSend.can_id  = id; // ID CANbus
   canMsgSend.can_dlc = 8;     // Tamanho da mensagem em bytes
   canMsgSend.data[0] = 0x2B;  // Controle
   canMsgSend.data[1] = 0x18;  // Segundo byte do indice
   canMsgSend.data[2] = 0x32;  // Primeiro byte do indice
   canMsgSend.data[3] = 0x00;  // sub-indice
   canMsgSend.data[4] = 0xFF;  //
   canMsgSend.data[5] = 0x7F;  //
   canMsgSend.data[6] = 0x00;  //
   canMsgSend.data[7] = 0x00;  //
}
 
