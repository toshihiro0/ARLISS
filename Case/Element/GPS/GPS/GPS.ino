#include <SoftwareSerial.h>

SoftwareSerial mySerial(5,4); //RX,TX

void setup()
{
    Serial.begin(57600);
    while(!Serial){}
    mySerial.begin(9600);
}

void loop()
{
    if(mySerial.available()){
        Serial.write(mySerial.read());
    }
}
