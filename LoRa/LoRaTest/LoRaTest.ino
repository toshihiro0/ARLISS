#include<SoftwareSerial.h>
SoftwareSerial LoRa_ss(2,3);

void setup()
{
   Serial.begin(57600);
   LoRa_ss.begin(19200);
}

int number = 0;
void loop() 
{
   if(LoRa_ss.available()){
      Serial.write(LoRa_ss.read());
   }
   if(Serial.available()){
      LoRa_ss.write(Serial.read());
   }
   /*LoRa_ss.print("hogehoge");
   LoRa_ss.print(number);
   LoRa_ss.print("\r");
   ++number;
   delay(1000);*/
}
