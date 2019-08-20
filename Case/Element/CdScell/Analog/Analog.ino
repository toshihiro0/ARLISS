void setup()
{
    Serial.begin(57600);
    analogReference(DEFAULT);
}

int analogpin[3] = {0,6,7};

void loop()
{
    int i;
    for(i = 0;i < 3;++i){
        Serial.println(analogRead(analogpin[i]));
        delay(500);
    }
}
