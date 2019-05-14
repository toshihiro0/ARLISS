#include <TinyGPS++.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;
SoftwareSerial mySerial(3,4); //RX,TX

void setup()
{
    Serial.begin(57600);
    while(!Serial){}
    mySerial.begin(9600);
}

void loop()
{
    while (mySerial.available() > 0){
        char c = mySerial.read();
        //Serial.print(c);
        gps.encode(c);
        if (gps.location.isUpdated()){
            Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
            Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
            Serial.print("ALT="); Serial.println(gps.altitude.meters());
        }
    }
}
