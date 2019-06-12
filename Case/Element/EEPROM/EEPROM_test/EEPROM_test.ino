#include <EEPROM.h>
void setup()
{
  pinMode(7,OUTPUT);
  Serial.begin(57600);
  int i;
  unsigned long time1 = millis();
  for(i = 0;i < 256;++i){
    EEPROM.write(i,i);
  }
  unsigned long time2 = millis();
  Serial.println(time2-time1);
  digitalWrite(7,HIGH);
  delay(1000);
  digitalWrite(7,LOW);
  int value;
  for(i = 0;i < 256;++i){
    value = EEPROM.read(i);
    Serial.println(value);
  }
}

void loop()
{
  // put your main code here, to run repeatedly:

}
