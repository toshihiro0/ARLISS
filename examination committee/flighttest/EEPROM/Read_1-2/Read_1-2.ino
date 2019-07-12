#include <EEPROM.h>

void setup()
{
    Serial.begin(57600);
}

void loop()
{
    int i;
    float pitch;
    for(i = 0;i <= 5;++i){
        Serial.println(EEPROM.read(i));
    }
    while(true){}
}
