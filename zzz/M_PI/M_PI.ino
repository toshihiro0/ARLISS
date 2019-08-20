#include <math.h>

void setup()
{
    Serial.begin(57600);
}

void loop()
{
    Serial.println(M_PI*1000000);
}
