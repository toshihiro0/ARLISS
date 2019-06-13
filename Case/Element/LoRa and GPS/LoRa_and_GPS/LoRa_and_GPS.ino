#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define LoRa_sw 6 //LoRaの電源ピン
#define LoRa_rst 7 //LoRaのRstピン

TinyGPSPlus gps;
SoftwareSerial GPS_UART(5,4); //RX,TX,GPS通信用
//SoftwareSerial LoRa(8,9);

void setup()
{
    pinMode(LoRa_sw,OUTPUT); //LoRaの通信
    digitalWrite(LoRa_sw,HIGH); //ここを変えること！！！
    pinMode(LoRa_rst,OUTPUT);
    digitalWrite(LoRa_rst,HIGH);

    Serial.begin(19200);
    GPS_UART.begin(9600); //GPSとの通信
    //LoRa.begin(19200); //Loraとの通信
    delay(1200);
}

void loop()
{
    while (GPS_UART.available() > 0){
        char c = GPS_UART.read();
        gps.encode(c);
        if (gps.location.isUpdated()){
            Serial.print("LAT=");delay(100);Serial.println(gps.location.lat(), 6);delay(100);Serial.print("\r");delay(500);
            Serial.print("LONG=");delay(100);Serial.println(gps.location.lng(), 6);delay(100);Serial.print("\r");delay(500);
            Serial.print("ALT=");delay(100);Serial.println(gps.altitude.meters(),6);delay(100);Serial.print("\r");delay(500);
        }
    }
}
