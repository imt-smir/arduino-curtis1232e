#ifndef CANCURTIS_H
#define CANCURTIS_H
 
#include <Arduino.h>
#include <mcp2515.h>
 
class Curtis
{
public:
   Curtis(int ID);
   void  Throttle();
 
private:
   int ID;
};
 
#endif
