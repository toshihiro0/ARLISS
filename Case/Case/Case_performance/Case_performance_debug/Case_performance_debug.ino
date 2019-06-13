#include <SoftwareSerial.h>
#include <SparkFunBME280.h>
#include <TinyGPS++.h>
#include <EEPROM.h>

//#defineとstatic const のどちらが良いか僕にはわからない。以下ピン設定
//#defineのがメモリを食わないので、#defineにしておきます
//EEPROMによる再起動の冗長系をまだ入れていません。
//時間による冗長系をまだ入れていません。

//Cdsセル、0mでの気圧、気温確認、プログラム開始確認！！


#define nichrome_pin_1 2 //ニクロム線1つめ
#define nichrome_pin_2 3 //ニクロム線2つめ
#define SPI_CS_PIN 10 //気圧センサ

#define LoRa_sw 6 //LoRaの電源ピン
#define LoRa_rst 7 //LoRaのRstピン

static const float airpressure_on_the_ground = 101540.265; //高度計算用の地上の気圧(Pa)
static const float temperature_on_the_ground = 23.82; //高度計算用の地上の気温(℃)
static const float release_height = 2000; //切り離し高度(m)

int EEPROM_address = 0; //EEPROM用のaddress

SoftwareSerial GPS_UART(5,4); //RX,TX,GPS通信用
BME280 air_pressure_sensor; //気圧センサBME280
TinyGPSPlus gps; //GPS

void cds(void); //cdsセンサーの明暗判定
float heightjudge(void); //気圧センサの高度判定
void nichromecut(void); //ケーシング展開
void senttoLora(float);
void gps_transmission(void); //GPS情報の送信
void LoRa_reset(void);

void setup()
{
    analogReference(DEFAULT); //Cdsセルの電圧読み取り

    air_pressure_sensor.beginSPI(SPI_CS_PIN); //気圧センサとのSPI通信

    pinMode(nichrome_pin_1,OUTPUT); //以下ニクロム線
    digitalWrite(nichrome_pin_1,LOW);
    pinMode(nichrome_pin_2,OUTPUT);
    digitalWrite(nichrome_pin_2,LOW);

    pinMode(LoRa_sw,OUTPUT); //LoRaの通信
    digitalWrite(LoRa_sw,HIGH); //ここを変えること！！！
    pinMode(LoRa_rst,OUTPUT);
    digitalWrite(LoRa_rst,HIGH);

    GPS_UART.begin(9600); //GPSとの通信

    Serial.begin(19200); //Loraとの通信
}

void loop()
{
    float height; //高度判定
    delay(1200); //LoRa起動後は1200ms待つ必要がある
    Serial.print("The program has started.\r"); //本番もここら辺のdelay必要か?
    delay(5000);
    Serial.print("The process has started.\r");
    cds(); //明暗の判定
    //digitalWrite(LoRa_sw,HIGH); //ロケットから放出されたので、通信を開始してOK
    //Serial.begin(19200); //通信開始には多少待つ必要があるみたいだけど...
    height = heightjudge(); //高度判定
    nichromecut(); //ニクロム線カット
    senttoLora(height);
    gps_transmission(); //LoRaからGPS情報送信、ずっとこの中
}

void cds()
{
    int i,sum = 0;
    static const int analogpin = 7;
    static const int judge_value = 750;
    int judge_times = 0;
    boolean first = true;     
    while(judge_times < 3){ //3回連続OKでwhile抜ける
        for(i = 0;i < 5;++i){
            sum += analogRead(analogpin); //アナログピンの読み取り
            delay(20); //少しずつ遅らせて取らないと、何回も計測する意味が無い
        }
        int value = sum/5;
        if(value > judge_value){ //暗い時はVoltageが小さい、明るい時はVoltageが大きい
            sum = 0;
            ++judge_times;
            if(judge_times == 3){
                EEPROM.write(EEPROM_address,2);
                ++EEPROM_address;
                EEPROM.write(EEPROM_address,value);
                ++EEPROM_address;
            }
            continue; //もう一度
        }else{
            judge_times = 0;
            sum = 0; //初期化
            if(first){
                EEPROM.write(EEPROM_address,1);
                ++EEPROM_address;
                EEPROM.write(EEPROM_address,value);
                ++EEPROM_address;
                first = false;
            }
            continue; //また計測
        }
    }
    return;
}

float heightjudge()
{
    static const float temperature_correction = 273.15; //℃↔Kの変換
    float pressure;
    float height;
    int i;
    for(i = 0;i < 5;++i){ //値の取り始めは値がおかしい。
        air_pressure_sensor.readTempC(); //よくわからないがこれを無くすと気圧の値がおかしくなる?????。
        pressure = air_pressure_sensor.readFloatPressure();
        delay(100);
    }
    while(true){
        air_pressure_sensor.readTempC(); //よくわからないがこれを無くすと気圧の値がおかしくなる?????。
        pressure = air_pressure_sensor.readFloatPressure();
        height = (1-pow(pressure/airpressure_on_the_ground,0.19035714))/0.0065*(temperature_on_the_ground+temperature_correction);
        if(height <= release_height){
            air_pressure_sensor.setMode(MODE_SLEEP); //気圧センサスリープモード(電源消費を抑える)
            EEPROM.write(EEPROM_address,3);
            ++EEPROM_address;
            EEPROM.write(EEPROM_address,(int)height);
            ++EEPROM_address;
            return height;
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
    EEPROM.write(EEPROM_address,4);
    ++EEPROM_address;
    digitalWrite(nichrome_pin_2,HIGH);
    delay(3000);
    digitalWrite(nichrome_pin_2,LOW);
    EEPROM.write(EEPROM_address,5);
    ++EEPROM_address;
    return;
}

void senttoLora(float height)
{
    Serial.print("Case has released.\r");
    delay(500);
    Serial.print("The pointed height has arrived.\r");
    delay(500);
    Serial.print("Height: ");
    delay(500);
    Serial.print(height);
    delay(500);
    Serial.print("\r");
    delay(500);
    Serial.print("The aircraft has released.\r");
    delay(500);
    return;
}

void gps_transmission()
{
    EEPROM.write(EEPROM_address,6);
    ++EEPROM_address;
    while(true){
        while (GPS_UART.available() > 0){
            char c = GPS_UART.read();
            gps.encode(c);
            if(gps.location.isUpdated()){
                Serial.print("LAT=");delay(100);Serial.print(gps.location.lat(), 6);delay(100);Serial.print("\r");delay(500);
                Serial.print("LONG=");delay(100);Serial.print(gps.location.lng(), 6);delay(100);Serial.print("\r");delay(500);
                Serial.print("ALT=");delay(100);Serial.println(gps.altitude.meters());delay(100);Serial.print("\r");delay(500);
            }
        }
    }
}

void LoRa_reset()
{
    digitalWrite(LoRa_rst,LOW);
    delay(1); //1msで十分
    digitalWrite(LoRa_rst,HIGH);
}