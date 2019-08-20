#include <EEPROM.h>

void setup()
{
    Serial.begin(57600);
}

void loop()
{
    float latitude,longtitude,altitude;
    unsigned time1,time2,time3;

    EEPROM.get(0,latitude);
    Serial.println(latitude);
    EEPROM.get(4,time1);
    Serial.println(time1);

    EEPROM.get(8,longtitude);
    Serial.println(longtitude);
    EEPROM.get(12,time2);
    Serial.println(time2);

    EEPROM.get(16,altitude);
    Serial.println(altitude);
    EEPROM.get(20,time3);
    Serial.println(time3);
    while(true){}
}