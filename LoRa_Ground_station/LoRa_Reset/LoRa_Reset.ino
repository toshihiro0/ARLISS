#include<SoftwareSerial.h>

#define LoRa_Rx 4
#define LoRa_Tx 2
#define LoRa_rst 3
#define button_pin 8

SoftwareSerial LoRa_ss(LoRa_Rx,LoRa_Tx);

void setup()
{
   	pinMode(button_pin,INPUT_PULLUP);
    pinMode(LoRa_rst,OUTPUT);
    digitalWrite(LoRa_rst,HIGH);
    
   	Serial.begin(57600);
   	LoRa_ss.begin(19200);
}

void loop() 
{
    if(LoRa_ss.available()){
    	Serial.write(LoRa_ss.read());
    }
    if(Serial.available()){
        LoRa_ss.write(Serial.read());
    }
	if(digitalRead(button_pin) == LOW){
        while(digitalRead(button_pin) == LOW){}
		digitalWrite(LoRa_rst,LOW);
		delay(1);
        digitalWrite(LoRa_rst,HIGH);
	}
}
