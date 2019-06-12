#include <SparkFunBME280.h>

const int SPI_CS_PIN = 10;
BME280 sensor;
void setup()
{
    Serial.begin(115200);
    sensor.beginSPI(SPI_CS_PIN);
}

void loop()
{
    Serial.print("Temp: ");
    Serial.print(sensor.readTempC(), 2);

    Serial.print(" °C, Humidity: ");
    Serial.print(sensor.readFloatHumidity(), 2);

    Serial.print(" %, Pressure: ");
    Serial.print(sensor.readFloatPressure() / 100.0, 1);
    Serial.println(" hPa");

    delay(1000);
}
