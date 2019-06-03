#include<SoftwareSerial.h>
SoftwareSerial LoRa_ss(2,3);

void setup()
{
   	pinMode(4,OUTPUT);
   	pinMode(5,OUTPUT);
   	digitalWrite(4,HIGH);
   	digitalWrite(5,HIGH);
	pinMode(7,OUTPUT);
	digitalWrite(7,HIGH);
   	pinMode(8,INPUT);
   	Serial.begin(57600);
   	LoRa_ss.begin(19200);
}

int number = 0;
void loop() 
{
    if(LoRa_ss.available()){
    	Serial.write(LoRa_ss.read());
    }
    if(Serial.available()){
        LoRa_ss.write(Serial.read());
    }
	if(digitalRead(8) == HIGH){
    while(digitalRead(8) == HIGH){}
		digitalWrite(7,LOW);
		delay(1);
    digitalWrite(7,HIGH);
	}
   /*while(true){
    LoRa_ss.print("hogehoge");
    LoRa_ss.print(number);
    LoRa_ss.print("\r");
    ++number;
    delay(1000);
   }*/
   
}
