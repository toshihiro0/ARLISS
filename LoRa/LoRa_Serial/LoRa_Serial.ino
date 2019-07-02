#include <SoftwareSerial.h>

#define LoRa_sw 6
#define LoRa_rst 7

void setup()
{
   pinMode(LoRa_sw,OUTPUT);
   digitalWrite(LoRa_sw,HIGH);
   pinMode(LoRa_rst,OUTPUT);
   digitalWrite(LoRa_rst,HIGH);
   Serial.begin(19200);
   delay(2000);
}

int number = 0;
void loop() 
{
  Serial.print("hogehoge");
  Serial.print(number);
  Serial.print("\r");
	++number;
	delay(1000);
}
