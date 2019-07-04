#include <SoftwareSerial.h>
#include <string.h>

SoftwareSerial LoRa(8,9);

void setup()
{
    pinMode(6,OUTPUT);
    pinMode(7,OUTPUT);
    digitalWrite(6,HIGH);
    digitalWrite(7,HIGH);
    pinMode(2,OUTPUT); //LEDの点滅でコード開始を確認
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
        LoRa.print("return\r");
        delay(400);
        if(strstr(buf,"cutoff")!= NULL){
            LoRa.print("kakunin\r");
            delay(100);
        }
    }
}
void LoRa_recv(char *buf)
{
    char *string_pointer = buf;
    int time1 = millis();
    int time2;
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
        time2 = millis();
        if((time2-time1) > 10000){
            digitalWrite(2,HIGH);
            delay(2000);
            digitalWrite(2,LOW);
            time1 = time2;
        }
    }
}
