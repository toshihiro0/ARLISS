#include <SoftwareSerial.h>
#include <SparkFunBME280.h>
#include <TinyGPS++.h>

//#defineとstatic const のどちらが良いか僕にはわからない。以下ピン設定

static const int nichrome_pin_1 = 4; //ニクロム線1つめ
static const int nichrome_pin_2 = 5; //ニクロム線2つめ
static const int SPI_CS_PIN = 10; //気圧センサ
//static const int LoRa_sw = 7; //LoRaの電源ピン
//static const int LoRa_rst = 6; //LoRaのRstピン、リセットのために作ってはあるけど、これ使うのか?

static const float airpressure_on_the_ground = 101540.265; //高度計算用の地上の気圧(Pa)
static const float temperature_on_the_ground = 23.82; //高度計算用の地上の気温(℃)
static const float release_height = 2000; //切り離し高度(m)

SoftwareSerial GPS_UART(2,3); //RX,TX,GPS通信用
BME280 air_pressure_sensor; //気圧センサBME280
TinyGPSPlus gps; //GPS

void cds(void); //cdsセンサーの明暗判定
void heightjudge(void); //気圧センサの高度判定
void nichromecut(void); //ケーシング展開
void gps_transmission(void); //GPS情報の送信

void setup()
{
    analogReference(DEFAULT); //Cdsセルの電圧読み取り

    air_pressure_sensor.beginSPI(SPI_CS_PIN); //気圧センサとのSPI通信

    pinMode(nichrome_pin_1,OUTPUT); //以下ニクロム線
    digitalWrite(nichrome_pin_1,LOW);
    pinMode(nichrome_pin_2,OUTPUT);
    digitalWrite(nichrome_pin_2,LOW);

    //pinMode(LoRa_sw,OUTPUT); //LoRaの通信
    //digitalWrite(LoRa_sw,LOW);

    GPS_UART.begin(9600); //GPSとの通信
}

void loop()
{
    cds(); //明暗の判定
    //digitalWrite(LoRa_sw,HIGH); //ロケットから放出されたので、通信を開始してOK
    //Serial.begin(19200); //通信開始には多少待つ必要があるみたいだけど...
    heightjudge(); //高度判定
    nichromecut(); //ニクロム線カット
    gps_transmission(); //LoRaからGPS情報送信、永遠にここにいる
}

void cds()
{
    int i,cds_sum = 0;
    int analogpin = 7;
    int cds_judge_value = 200;
    int cds_judge_times = 0;     
    while(cds_judge_times < 3){ //3回連続OKでwhile抜ける
        for(i = 0;i < 5;++i){
            cds_sum += analogRead(analogpin); //アナログピンの読み取り
            delay(2); //少しずつ遅らせて取らないと、何回も計測する意味が無い
        }
        int cds_value = cds_sum/5;
        if(cds_value > cds_judge_value){ //暗い時はVoltageが小さい、明るい時はVoltageが大きい
            continue; //もう一度
        }else{
            cds_judge_times = 0;
            cds_sum = 0; //初期化
            continue; //また計測
        }
    }
    return;
}

void heightjudge()
{
    static const float temperature_correction = 273.15; //℃↔Kの変換
    float pressure;
    float height;
    while(true){
        pressure = air_pressure_sensor.readFloatPressure();
        height = (1-pow(pressure/airpressure_on_the_ground,0.19035714))/0.0065*(temperature_on_the_ground+temperature_correction);
        if(height <= release_height){
            air_pressure_sensor.setMode(MODE_SLEEP); //気圧センサスリープモード(電源消費を抑える)
            return;
        }else{
            delay(10);
            continue;
        }
    }
}

void nichromecut()
{
    digitalWrite(nichrome_pin_1,HIGH);
    delay(3000);
    digitalWrite(nichrome_pin_1,LOW);
    delay(10); //とりあえず
    digitalWrite(nichrome_pin_2,HIGH);
    delay(3000);
    digitalWrite(nichrome_pin_2,LOW);
    delay(10);
    return;
}

void gps_transmission()
{
    while(true){
        while (GPS_UART.available() > 0){
            char c = GPS_UART.read();
            gps.encode(c);
            /*if(gps.location.isUpdated()){
                Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
                Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
                Serial.print("ALT="); Serial.println(gps.altitude.meters());
            }*/
        }
    }
}
