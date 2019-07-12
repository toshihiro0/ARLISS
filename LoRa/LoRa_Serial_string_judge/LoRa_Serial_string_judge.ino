#include <string.h>

void setup()
{
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
    digitalWrite(6,HIGH);
    digitalWrite(7,HIGH);
    Serial.begin(19200); //LoRa
    delay(2000);
}

void loop()
{
    while(true){
        char buf[128];
        LoRa_recv(buf);
        Serial.print("return\r");
        delay(400);
        if(strstr(buf,"cutoff")!= NULL){
            Serial.print("kakunin\r");
            delay(100);
        }
    }
}
void LoRa_recv(char *buf)
{
    Serial.print("in\r");
    delay(30);
    while (true) {
        while (Serial.available() > 0) {
            *buf++ = Serial.read();
            if(*(buf-3) == 'O' && *(buf-2) == 'K' && *(buf-1) == '\r'){
                continue;
            }else if (*(buf-1) == '\r'){
                *buf = '\0';
                return;
            }
        }
    }
}
