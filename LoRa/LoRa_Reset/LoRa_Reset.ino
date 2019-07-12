#include<SoftwareSerial.h>
SoftwareSerial LoRa_ss(8,9);

void setup()
{
   	pinMode(5,INPUT_PULLUP);

    pinMode(6,OUTPUT);
    digitalWrite(6,HIGH);
    pinMode(7,OUTPUT);
    digitalWrite(7,HIGH);
    
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
	if(digitalRead(5) == LOW){
    while(digitalRead(5) == LOW){}
		digitalWrite(7,LOW);
		delay(1);
    digitalWrite(7,HIGH);
	}
}
