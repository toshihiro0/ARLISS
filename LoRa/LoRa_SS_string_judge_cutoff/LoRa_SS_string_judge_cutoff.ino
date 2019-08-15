#include <SoftwareSerial.h>
#include <string.h>

#define nichromcutpin_1 2
#define nichromcutpin_2 3

SoftwareSerial LoRa(8,9);

void setup()
{
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
    digitalWrite(6,HIGH);
    digitalWrite(7,HIGH);
    //Serial.begin(57600); //Main serial port for console output

    pinMode(nichromcutpin_1,OUTPUT);
    pinMode(nichromcutpin_2,OUTPUT);
    LoRa.begin(19200);
    delay(2000);
}

void loop()
{
    while(true){
        char buf[128];
        LoRa_recv(buf);
        LoRa.print("return\r");
        delay(400);
        if(strstr(buf,"cutoff")!= NULL){
            nichromcut();
        }
    }
}

void LoRa_recv(char *buf)
{
    while (true) {
        while (LoRa.available() > 0) {
            *buf++ = LoRa.read();
            if(*(buf-3) == 'O' && *(buf-2) == 'K' && *(buf-1) == '\r'){
                continue;
            }else if (*(buf-1) == '\r'){
                *buf = '\0';
                return;
            }
        }
    }
}

void nichromcut()
{
    digitalWrite(nichromcutpin_1,HIGH);
    delay(4000);
    digitalWrite(nichromcutpin_1,LOW);
    delay(100);
    digitalWrite(nichromcutpin_2,HIGH);
    delay(4000);
    digitalWrite(nichromcutpin_2,LOW);
    while(true){}
}