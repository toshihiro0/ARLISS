#include <SparkFunBME280.h>

const int SPI_CS_PIN = 10;
BME280 sensor;

static const float pressure_on_the_ground = 101301.21; //高度計算用の地上の気圧(Pa)
static const float temperature_on_the_ground = 29.60; //高度計算用の地上の気温(℃)
static const float temperature_correction = 273.15;

void setup()
{
    Serial.begin(115200);
    sensor.beginSPI(SPI_CS_PIN);
}

float temepratureK;
float pressure;
float height;

void loop()
{
    Serial.print("Temp: ");
    Serial.print(sensor.readTempC());

    Serial.print(" °C, Humidity: ");
    Serial.print(sensor.readFloatHumidity());

    Serial.print(" %, Pressure: ");
    Serial.print(sensor.readFloatPressure());
    Serial.println(" Pa");
    
    pressure = sensor.readFloatPressure();
    height = (1-pow(pressure/pressure_on_the_ground,0.19035714))/0.0065*(temperature_on_the_ground+temperature_correction);
    Serial.print("Height: ");
    Serial.print(height);
    Serial.println(" m");

    Serial.print("Altitude: ");
    Serial.print(sensor.readFloatAltitudeMeters(), 0);
    Serial.println(" m");

    delay(1000);
}
