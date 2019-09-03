#include <EEPROM.h>

void setup()
{
    Serial.begin(57600);
    EEPROM.write(0,1);
    if(EEPROM.read(0) == 1){
        return;
    }else{
        delay(10000);
    }
}

void loop()
{
    Serial.write("hogehoge\r");
    delay(500);

}
