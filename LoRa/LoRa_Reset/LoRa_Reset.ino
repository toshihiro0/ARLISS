#include<SoftwareSerial.h>
SoftwareSerial LoRa_ss(17,19);

void setup()
{
   	pinMode(5,INPUT_PULLUP);

    pinMode(6,OUTPUT);
    digitalWrite(6,HIGH);
    pinMode(18,OUTPUT);
    digitalWrite(18,HIGH);
    
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
		digitalWrite(18,LOW);
		delay(1);
    digitalWrite(18,HIGH);
	}
}
