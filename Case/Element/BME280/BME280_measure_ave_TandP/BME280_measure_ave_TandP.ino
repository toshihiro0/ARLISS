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
    //long time1,time2;
    int i;
    float temp_sum = 0;
    float pressure_sum = 0;
    //time1 = millis();
    for(i = 0;i < 10000;++i){
        temp_sum += sensor.readTempC();
        pressure_sum += sensor.readFloatPressure();
    }
    //time2 = millis();
    Serial.print("Temp: ");
    Serial.print(temp_sum/i);
    Serial.println(" °C");
    Serial.print("Pressure: ");
    Serial.print(pressure_sum/i);
    Serial.println(" hPa");
    //Serial.println(time2-time1);
}
