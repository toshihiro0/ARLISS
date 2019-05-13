void setup()
{
    Serial.begin(57600);
    analogReference(DEFAULT);
}

int analogpin = 7;

void loop()
{
    Serial.print(analogRead(analogpin));
    Serial.print("\n");
    delay(20);
}
