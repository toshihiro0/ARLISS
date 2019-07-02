#include <EEPROM.h>

void setup()
{
  Serial.begin(57600);
}

void loop()
{
  float height;
  EEPROM.get(0,height);
  Serial.println(height);
  delay(1000);
}
