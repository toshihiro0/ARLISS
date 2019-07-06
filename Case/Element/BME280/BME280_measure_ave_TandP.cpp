#include <SparkFunBME280.h>

const int SPI_CS_PIN = 10;
BME280 sensor;
void setup()
{
    Serial.begin(115200);
    sensor.beginSPI(SPI_CS_PIN);
}

long measurenumber = 0;

void loop()
{
    long time1,time2;
    float temp_sum = 0;
    float pressure_sum = 0;
    while(true){
        time1 = millis();
        for(i = 0;i < 1000;++i){
            temp_sum += sensor.readTempC();
            pressure_sum += sensor.readFloatPressure();
            delay(1); //時間によってどう変化するかはちょっと見たい。
        }
        time2 = millis();
        measurenumber += i;
        Serial.print("Temp: ");
        Serial.print(temp_sum/measurenumber);
        Serial.write(" °C");
        Serial.print("Pressure: ")
        Serial.print(pressure_sum/pressuresum);
        Serial.write(" hPa")
        Serial.println(time2-time1);
    }
}