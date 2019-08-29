#include<SoftwareSerial.h>

#define LoRa_Rx 4
#define LoRa_Tx 2
#define LoRa_rst 3
#define button_pin 8

SoftwareSerial LoRa_ss(LoRa_Rx,LoRa_Tx);

void setup()
{
     pinMode(button_pin,INPUT_PULLUP);
    pinMode(LoRa_rst,OUTPUT);
    digitalWrite(LoRa_rst,HIGH);
    
    Serial.begin(57600);
    LoRa_ss.begin(19200);
    delay(2000);
}

void loop() 
{
    LoRa_ss.write("hogehoge\r\n");
    delay(500);
}
