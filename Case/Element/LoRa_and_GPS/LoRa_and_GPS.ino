#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define LoRa_sw 6 //LoRaの電源ピン
#define LoRa_rst 7 //LoRaのRstピン

TinyGPSPlus gps;
SoftwareSerial GPS_UART(5,4); //RX,TX,GPS通信用
SoftwareSerial LoRa(8,9); //RX,TX,LoRa通信用

void setup()
{
    pinMode(LoRa_sw,OUTPUT); //LoRaの通信
    digitalWrite(LoRa_sw,HIGH); //ここを変えること！！！
    pinMode(LoRa_rst,OUTPUT);
    digitalWrite(LoRa_rst,HIGH);

    Serial.begin(57600);
    GPS_UART.begin(9600); //GPSとの通信
    LoRa.begin(19200); //Loraとの通信
}

void loop()
{
    float lat;
    float lng;
    float alt;
    while (mySerial.available() > 0){
        char c = mySerial.read();
        gps.encode(c);
        if (gps.location.isUpdated()){
            lat = gps.location.lat();
            lng = gps.location.lng();
            alt = gps.altitude.meters();
            Serial.print("LAT="); Serial.println(lat, 6);
            Serial.print("LONG="); Serial.println(lng, 6);
            Serial.print("ALT="); Serial.println(lng,6);
            LoRa.print("LAT=");delay(500);LoRa.print(lat, 6);delay(500);LoRa.print("\r");delay(500);
            LoRa.print("LONG=");delay(500);LoRa.print(lng, 6);delay(500);LoRa.print("\r");delay(500);
            LoRa.print("ALT=");delay(500);LoRa.println(alt, 6);delay(500);LoRa.print("\r");delay(500);
        }
    }
}