#include <EEPROM.h>

void setup()
{
    Serial.begin(57600);
}

void loop()
{
    int i;
    float lat,lon,alt;
    for(i = 0;i <= 6;++i){
        Serial.println(EEPROM.read(i));
    }
    EEPROM.get(i,lat);
    Serial.println(lat);
    i += 4;
    EEPROM.get(i,lon);
    Serial.println(lon);
    i += 4;
    EEPROM.get(i,alt);
    Serial.println(alt);
    i += 4;
    Serial.println(EEPROM.read(i));
    while(true){}
}
