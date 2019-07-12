#include <EEPROM.h>

void setup()
{
    Serial.begin(57600);
}

void loop()
{
    int i;
    float pitch;
    for(i = 0;i <= 1;++i){
        Serial.println(EEPROM.read(i));
    }
    EEPROM.get(i,pitch);
    Serial.println(pitch);
    for(i = 6;i <= 8;++i){
        Serial.println(EEPROM.read(i));
    }
    while(true){}
}