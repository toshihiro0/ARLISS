#include <SoftwareSerial.h>
#include <SparkFunBME280.h>
#include <TinyGPS++.h>
#include <EEPROM.h> //EEPROMに保存

#define nichrome_pin_1 2
#define nichrome_pin_2 3
#define SPI_CS_PIN 10 //気圧センサ

#define LoRa_sw 6 //LoRaの電源ピン
#define LoRa_rst 7 //LoRaのRstピン

#define airpressure_on_the_ground 100210.78 //高度計算用の地上の気圧(Pa)
static const float temperature_on_the_ground = 23.66; //高度計算用の地上の気温(℃)
static const float temperature_correction = 273.15;

unsigned long time1;
unsigned long time2;

SoftwareSerial LoRa(8,9);

BME280 air_pressure_sensor; //BME280

TinyGPSPlus gps; //GPS

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
    LoRa.begin(19200); //LoRaとの通信
    delay(2000); //LoRaの起動待ち
}

void loop()
{
    int i;
    float height;
    while(true){
        char buf[128];
        LoRa_recv(buf);
        if(strstr(buf,"cutoff")!= NULL){
            break;
        }
    }
    LoRa.write("config\r");
    delay(100);
    digitalWrite(LoRa_rst,LOW);
    delay(100);
    digitalWrite(LoRa_rst,HIGH);
    delay(2000);
    LoRa.write("2\r");
    delay(50);
    LoRa.write("g 0\r");
    delay(50);
    LoRa.write("q 2\r");
    delay(50);
    LoRa.write("save\r");
    delay(100);
    LoRa.write("start\r");
    delay(100);
    gps_transmission();
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

void LoRa_recv(char *buf)
{
    while (true) {
        while (LoRa.available() > 0) {
            *buf++ = LoRa.read();
            if(*(buf-3) == 'O' && *(buf-2) == 'K' && *(buf-1) == '\r'){
                continue;
            }else if (*(buf-1) == '\r'){
                *buf = '\0';
                return;
            }
        }
    }
}

void gps_transmission()
{
    Serial.begin(9600); //GPSとの通信 

    while(true){
        while (Serial.available() > 0){
            char c = Serial.read();
            gps.encode(c);
            if(gps.location.isUpdated()){
                int i;
                float lat,lon,alt;
                lat = gps.location.lat();
                lon = gps.location.lng();
                alt = gps.altitude.meters();
                LoRa.print("LAT=");delay(100);LoRa.println(lat,10);delay(500);
                LoRa.print("LONG=");delay(100);LoRa.println(lon,10);delay(500);
                LoRa.print("ALT=");delay(100);LoRa.println(alt,10);delay(500);
                for(i = 28;i < 40;++i){
                    EEPROM.write(i,0);
                }
                EEPROM.put(28,lat);
                EEPROM.put(32,lon);
                EEPROM.put(36,alt);
            }
        }
    }
}
