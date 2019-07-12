#include <EEPROM.h>

void setup()
{
    Serial.begin(57600);
}

void loop()
{
    int i;
    float pitch;
    for(i = 0;i <= 3;++i){
        Serial.println(EEPROM.read(i));
    }
    EEPROM.get(i,pitch);
    Serial.println(pitch);
    for(i = 8;i <= 9;++i){
        Serial.println(EEPROM.read(i));
    }
    while(true){}
}