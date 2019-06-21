#include <SoftwareSerial.h>
#include <string.h>

SoftwareSerial LoRa(8,9);

void setup()
{
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
    digitalWrite(6,HIGH);
    digitalWrite(7,HIGH);
    pinMode(2,OUTPUT);
    digitalWrite(2,HIGH);
    delay(1000);
    digitalWrite(2,LOW);

    Serial.begin(57600); //Main serial port for console output
    LoRa.begin(19200);
}

void loop()
{
    while(true){
        char buf[128];
        LoRa_recv(buf);
        Serial.write("return");
        if(strstr(buf,"cutoff")!= NULL){
            Serial.write("kakunin");
        }
    }
}

int LoRa_recv(char *buf) {
    char *string_pointer = buf;
    int time1 = millis();
    int time2;
    while (true) {
        while (LoRa.available() > 0) {
            *buf++ = LoRa.read();
            Serial.write(*(buf-1));
            if (*(buf-1) == '\r'){
                *buf = '\0';
                return;
            }
        }
        time2 = millis();
        if((time2-time1) > 10000){
            digitalWrite(2,HIGH);
            delay(2000);
            digitalWrite(2,LOW);
            time1 = time2;
        }
    }
}
