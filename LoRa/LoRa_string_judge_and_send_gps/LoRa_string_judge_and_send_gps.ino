#include <SoftwareSerial.h>
#include <string.h>
#include <TinyGPS++.h> //電池は外しておくこと。コネクタを付けること

SoftwareSerial LoRa(8,9);
TinyGPSPlus gps;

void setup()
{
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
    digitalWrite(6,HIGH);
    digitalWrite(7,HIGH);
    LoRa.begin(19200);
    delay(2000);
}

void loop()
{
    while(true){
        char buf[128];
        LoRa_recv(buf);
        LoRa.print("return\r");
        delay(400);
        if(strstr(buf,"cutoff")!= NULL){
            LoRa.print("kakunin\r");
            delay(100);
        }else if(strstr(buf,"gps") != NULL){
            LoRa.print("Yes\r");
            delay(100);
            LoRa_send_gps();
        }
    }
}

void LoRa_recv(char *buf)
{
    while (true) {
        while (LoRa.available() > 0) {
            *buf++ = LoRa.read();
            if(*(buf-3) == 'O' && *(buf-2) == 'K' && *(buf-1) == '\r'){
                continue;
            }else if (*(buf-1) == '\r'){
                *buf = '\0';
                return;
            }
        }
    }
}

void LoRa_send_gps()
{
    Serial.begin(9600); //GPS
    while(true){
        Serial.print("in1\r");delay(30);
        while (Serial.available() > 0){
            char c = Serial.read();
            gps.encode(c);
            LoRa.print("in2\r");
            if (gps.location.isUpdated()){
                LoRa.print("in2\r");delay(100);
                LoRa.print("LAT=");delay(100);LoRa.print(gps.location.lat(), 6);delay(100);LoRa.print("\r");delay(500);
                LoRa.print("LONG=");delay(100);LoRa.print(gps.location.lng(), 6);delay(100);LoRa.print("\r");delay(500);
                LoRa.print("ALT=");delay(100);LoRa.print(gps.altitude.meters());delay(100);LoRa.print("\r");delay(500);
            }
        }
    }
    
}
