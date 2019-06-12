#include<SoftwareSerial.h>
SoftwareSerial LoRa_ss(2,3);

void setup()
{
   pinMode(4,OUTPUT);
   pinMode(5,OUTPUT);
   digitalWrite(4,HIGH);
   digitalWrite(5,HIGH);
   Serial.begin(57600);
   LoRa_ss.begin(19200);
}

int number = 0;
void loop() 
{
   if(LoRa_ss.available()){
      Serial.write(LoRa_ss.read());//受信したデータの読み込み
   }
   if(Serial.available()){
      LoRa_ss.write(Serial.read());//入力されたデータを送信
   }
//   LoRa_ss.print("hogehoge");
//   LoRa_ss.print(number);
//   LoRa_ss.print("\r");
//   ++number;
//   delay(1000);
   
}
