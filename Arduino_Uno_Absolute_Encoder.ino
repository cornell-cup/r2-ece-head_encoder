#include <SPI.h>
#include <stdint.h>

#define CS 10 //Chip or Slave select 

void setup()
{
  pinMode(CS,OUTPUT);//Slave Select
  digitalWrite(CS,HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  
  Serial.begin(115200);
  Serial.println("starting");
}

uint8_t SPI_T(uint8_t msg){

  
  digitalWrite(CS, LOW);
  delayMicroseconds(20);
  msg = SPI.transfer(msg);
  digitalWrite(CS, HIGH);

  return msg;
}

void loop()
{ 
  Serial.println(SPI_T(0x10));
  
  
  Serial.println("loop");
   
   SPI_T(0x10);
   
   uint8_t recieved = SPI_T(0x00);    //issue NOP to check if encoder is ready to send
   
   while (recieved != 0x10)    //loop while encoder is not ready to send
   {
     recieved = SPI_T(0x00);    //cleck again if encoder is still working 
     delay(2);    //wait a bit
     Serial.println(recieved, BIN);
   }
   
   uint8_t MSB = SPI_T(0x00);    //Recieve MSB
   uint8_t LSB = SPI_T(0x00);    // recieve LSB  

   uint16_t pos = (MSB<<8) & LSB;
   
   Serial.println(pos,BIN);
   delay(500);
}
