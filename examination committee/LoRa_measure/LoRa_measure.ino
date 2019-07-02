#include <SoftwareSerial.h>
#include <SparkFunBME280.h>
#include <EEPROM.h> //EEPROMに保存

#define nichrome_pin_1 2
#define nichrome_pin_2 3
#define SPI_CS_PIN 10 //気圧センサ

#define LoRa_sw 6 //LoRaの電源ピン
#define LoRa_rst 7 //LoRaのRstピン

#define airpressure_on_the_ground 100434.000 //高度計算用の地上の気圧(Pa)
static const float temperature_on_the_ground = 25.25; //高度計算用の地上の気温(℃)
static const float temperature_correction = 273.15;

BME280 air_pressure_sensor; //BME280

void setup()
{
    pinMode(nichrome_pin_1, OUTPUT);
    pinMode(nichrome_pin_2, OUTPUT);
    digitalWrite(nichrome_pin_1, LOW);
    digitalWrite(nichrome_pin_2, LOW);

    pinMode(LoRa_sw, OUTPUT);
    pinMode(LoRa_rst,OUTPUT);
    digitalWrite(LoRa_sw,HIGH);
    digitalWrite(LoRa_rst,HIGH);

    air_pressure_sensor.beginSPI(SPI_CS_PIN); //気圧センサとのSPI通信
    Serial.begin(19200); //LoRaとの通信
    delay(2000); //LoRaの起動待ち
}

void loop()
{
    while(true){
        int i;
        float height = height_judge();
        for(i = 0;i < 4;++i){
          EEPROM.write(i,0); //putのために中身をClearしておく
        }
        EEPROM.put(0,height); //EEPROMに保存
        Serial.print(height);delay(100);
        Serial.print("\r");delay(10);
        delay(1000);
    }
}

float height_judge()
{
    float pressure;
    float height;
    int i;
    for(i = 0;i < 5;++i){ //値の取り始めは値がおかしい。
        air_pressure_sensor.readTempC(); //よくわからないがこれを無くすと気圧の値がおかしくなる?????。
        pressure = air_pressure_sensor.readFloatPressure();
        delay(100);
    }
    air_pressure_sensor.readTempC(); //よくわからないがこれを無くすと気圧の値がおかしくなる?????。
    pressure = air_pressure_sensor.readFloatPressure();
    height = (1-pow(pressure/airpressure_on_the_ground,0.19035714))/0.0065*(temperature_on_the_ground+temperature_correction);
    return height;
}
