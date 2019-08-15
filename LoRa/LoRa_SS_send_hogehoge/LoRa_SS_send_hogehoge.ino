#include <SoftwareSerial.h>

#define LoRa_sw 6
#define LoRa_rst 7

SoftwareSerial LoRa(8,9);

void setup()
{
   pinMode(LoRa_sw,OUTPUT);
   digitalWrite(LoRa_sw,HIGH);
   pinMode(LoRa_rst,OUTPUT);
   digitalWrite(LoRa_rst,HIGH);
   LoRa.begin(19200);
   delay(2000); //LoRa起動待ち
}

int number = 0;
void loop() 
{
  LoRa.print("hogehoge");
  LoRa.print(number);
  LoRa.print("\r");
	++number;
	delay(3000);
}
