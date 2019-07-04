#include<SoftwareSerial.h>
SoftwareSerial LoRa_ss(8,9);

void setup()
{
   pinMode(6,OUTPUT);
   digitalWrite(6,HIGH);
   pinMode(7,OUTPUT);
   digitalWrite(7,HIGH);
   Serial.begin(57600);
   LoRa_ss.begin(19200);
   delay(2000);
}

// int number = 0;
void loop() 
{
   if(LoRa_ss.available()){
      Serial.write(LoRa_ss.read());//受信したデータの読み込み
   }
   if(Serial.available()){
      LoRa_ss.write(Serial.read());//入力されたデータを送信
   }
//   LoRa_ss.print("hogehoge");delay(100);
//   LoRa_ss.print(number);delay(10);
//   LoRa_ss.print("\r");delay(100);
//   ++number;
//   delay(1000);
   
}
