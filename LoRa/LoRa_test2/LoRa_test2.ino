#include <SoftwareSerial.h>
#include <string.h>
SoftwareSerial LoRa(8,9);

void setup()
{
   pinMode(6,OUTPUT);
   digitalWrite(6,HIGH);
   pinMode(7,OUTPUT);
   digitalWrite(7,HIGH);
   Serial.begin(57600);
   LoRa.begin(19200);

   pinMode(2,OUTPUT);
   digitalWrite(2,HIGH);
   delay(1000);
   digitalWrite(2,LOW);
}

void loop()
{
    LoRa_recv();
}

void LoRa_recv()
{
    while(true){
        char buf[128];
        char* temp; //開始を記憶
        while(true){ //ここの無限ループどうにかしたい
            temp = buf;
            while (LoRa.available() > 0){
                *temp++ = LoRa.read();
                Serial.write(*(temp-1));
                if (*(temp-1) == '\r') {
                    *temp = '\0';
                    break;
                }
            }
            if(strstr(buf, "cutoff") != NULL){
                digitalWrite(2,HIGH);
                delay(1000);
                digitalWrite(2,LOW);
            }else{
              continue;
            }
        }
    }
}
