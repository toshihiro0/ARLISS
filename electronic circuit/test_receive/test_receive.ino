#include <SoftwareSerial.h>

#define LoRa_sw 19
#define LoRa_rst 14
SoftwareSerial LoRa(11,10);

void setup()
{
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
