#include <SoftwareSerial.h>

#define LoRa_sw 6
#define LoRa_rst 18

SoftwareSerial LoRa(17,19);

void setup()
{
   pinMode(LoRa_sw,OUTPUT);
   digitalWrite(LoRa_sw,HIGH);
   pinMode(LoRa_rst,OUTPUT);
   digitalWrite(LoRa_rst,HIGH);
   LoRa.begin(19200);
   delay(2000); //LoRa起動待ち
}

void loop() 
{
  int i;
  for(i = 0;i < 10;++i){
    LoRa.print("hogehoge");
    LoRa.println(i);
    delay(1000);
  }
}
