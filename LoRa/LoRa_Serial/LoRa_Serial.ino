#include <SoftwareSerial.h>

//SoftwareSerial LoRa(8,9);

void setup()
{
   pinMode(6,OUTPUT);
   digitalWrite(6,HIGH);
   pinMode(7,OUTPUT);
   digitalWrite(7,HIGH);
   //LoRa.begin(19200);
   Serial.begin(19200);
   delay(1200);
}

int number = 0;
void loop() 
{
  //LoRa.print("hogehoge");
  //LoRa.print(number);
  //LoRa.print("\r");
  Serial.print("hogehoge");
  Serial.print(number);
  Serial.print("\r");
	++number;
	delay(1000);
}
