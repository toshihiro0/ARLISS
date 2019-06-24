void setup()
{
    Serial.begin(57600);
    analogReference(DEFAULT);
}

int analogpin = 7;

void loop()
{
    Serial.println(analogRead(analogpin));
    delay(20);
}
