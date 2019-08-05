#include <SoftwareSerial.h>

#define LoRa_sw 13
#define LoRa_rst 18
SoftwareSerial LoRa(17,19);

void setup()
{
    pinMode(LoRa_sw,OUTPUT);
    digitalWrite(LoRa_sw,HIGH);
    pinMode(LoRa_rst,OUTPUT);
    digitalWrite(LoRa_rst,HIGH);

    Serial.begin(57600);
    LoRa.begin(19200);
}

void loop() 
{
    if(LoRa.available()){
        Serial.write(LoRa.read());//受信したデータの読み込み
    }
    if(Serial.available()){
        LoRa.write(Serial.read());//入力されたデータを送信
    }
}
