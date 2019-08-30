void setup()
{
    pinMode(9,INPUT_PULLUP);
    pinMode(10,INPUT_PULLUP);
    Serial.begin(57600);
}

void loop()
{
    Serial.println(9);
    if(digitalRead(9) == HIGH){
        Serial.write("HIGH\n");
    }else{
        Serial.write("LOW\n");
    }
    Serial.println(10);
    if(digitalRead(10) == HIGH){
        Serial.write("HIGH\n");
    }else{
        Serial.write("LOW\n");
    }
    delay(1000);
}
