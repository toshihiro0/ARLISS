#include <SoftwareSerial.h>
#include <SparkFunBME280.h>
#include <TinyGPS++.h>
#include <EEPROM.h>

//Cdsセル、0mでの気圧、気温確認、プログラム開始確認！！

#define nichrome_pin_1 2 //ニクロム線1つめ
#define nichrome_pin_2 3 //ニクロム線2つめ
#define SPI_CS_PIN 10 //気圧センサ

#define LoRa_sw 6 //LoRaの電源ピン
#define LoRa_rst 7 //LoRaのRstピン
#define LoRa_RX 8
#define LoRa_TX 9

#define Cds_mode 0
#define height_measure_mode 1
#define nichrome_cut_mode 2 //LoRaの変更も含む。
#define GPS_mode 3

static const float airpressure_on_the_ground = 101058.02; //高度計算用の地上の気圧(Pa)
static const float temperature_on_the_ground = 28.68; //高度計算用の地上の気温(℃)
static const float release_height = 15.0; //切り離し高度(m)

int EEPROM_Address = 2;

unsigned long time_cds;
unsigned long time_height;
unsigned long time_nichromecut1_start;
unsigned long time_nichromecut1_end;
unsigned long time_nichromecut2_start;

SoftwareSerial LoRa(LoRa_RX,LoRa_TX);
BME280 air_pressure_sensor; //気圧センサBME280
TinyGPSPlus gps; //GPS

float height = 20000.0;

/*
void cds(void); //cdsセンサーの明暗判定
float heightjudge(void); //気圧センサの高度判定
void nichromecut(void); //ケーシング展開
void senttoLora(float);
void gps_transmission(void); //GPS情報の送信
void LoRa_reset(void);
*/

void setup()
{
    analogReference(DEFAULT); //Cdsセルの電圧読み取り

    air_pressure_sensor.beginSPI(SPI_CS_PIN); //気圧センサとのSPI通信

    pinMode(nichrome_pin_1,OUTPUT); //以下ニクロム線
    digitalWrite(nichrome_pin_1,LOW);
    pinMode(nichrome_pin_2,OUTPUT);
    digitalWrite(nichrome_pin_2,LOW);

    EEPROM.write(0,0);

    Serial.begin(9600); //GPSとの通信

    LoRa.begin(19200); //Loraとの通信
    if(EEPROM.read(1) == 0){ //EEPROMで初回起動かどうか調べる。
        delay(60000); //60sしたら開始(ロケット発射前にCdsセルが勘違いするのを防ぐ。)
        EEPROM.write(1,1);
    }
}

void loop()
{
    int case_condition = EEPROM.read(0);
    switch(case_condition){
        case Cds_mode:
            cds();
            time_cds = millis();
            EEPROM.put(EEPROM_Address,time_cds);
            EEPROM_Address += 4;

            pinMode(LoRa_sw,OUTPUT); 
            digitalWrite(LoRa_sw,HIGH);
            pinMode(LoRa_rst,OUTPUT);
            digitalWrite(LoRa_rst,HIGH);
            delay(2000);

            EEPROM.write(0,height_measure_mode);
            case_condition = height_measure_mode;
        break;

        case height_measure_mode:
            height = heightjudge(); //高度判定
            EEPROM.put(EEPROM_Address,height);
            EEPROM_Address += 4;

            time_height = millis();
            EEPROM.put(EEPROM_Address,time_height);
            EEPROM_Address += 4;

            EEPROM.write(0,nichrome_cut_mode);
            case_condition = nichrome_cut_mode;
        break;

        case nichrome_cut_mode:
            nichromecut(); //ニクロム線カット
            senttoLora(height);

            EEPROM.write(0,GPS_mode);
            case_condition = GPS_mode;
        break;


        case GPS_mode:
            gps_transmission(); //LoRaからGPS情報送信、ずっとこの中
        break;

        default:
        break;
    }  
}

void cds()
{
    int i,j,sum = 0;
    int analogpin[3] = {0,6,7};
    static const int judge_value = 600;
    int Voltage_measure_value;
    int judge_times = 0;     
    while(judge_times < 3){ //3回連続OKでwhile抜ける
        for(i = 0;i < 5;++i){
            int cds_voltage_value_temp_sum = 0;
            for(j = 0;j < 3;++j){
                cds_voltage_value_temp_sum += analogRead(analogpin[j]); //アナログピンの読み取り
            }
            cds_voltage_value_temp_sum /= 3;
            sum += cds_voltage_value_temp_sum;
            delay(60); //少しずつ遅らせて取らないと、何回も計測する意味が無い
        }
        Voltage_measure_value = sum/5;
        if(Voltage_measure_value > judge_value){ //暗い時はVoltageが小さい、明るい時はVoltageが大きい
            sum = 0;
            ++judge_times;
            delay(100); //60*3*5+100 = 1000で1秒ごとに取る。
            continue; //もう一度
        }else{
            judge_times = 0;
            sum = 0; //初期化
            delay(100);
            continue; //また計測
        }
    }
    EEPROM.put(EEPROM_Address,Voltage_measure_value);
    EEPROM_Address += 2;
    return;
}

float heightjudge()
{
    static const float temperature_correction = 273.15; //℃↔Kの変換
    float pressure;
    float height;
    int i;
    unsigned long time_cds,now_time;
    for(i = 0;i < 5;++i){ //値の取り始めは値がおかしい。
        air_pressure_sensor.readTempC(); //よくわからないがこれを無くすと気圧の値がおかしくなる?????。
        pressure = air_pressure_sensor.readFloatPressure();
        delay(100);
    }
    while(true){
        air_pressure_sensor.readTempC(); //よくわからないがこれを無くすと気圧の値がおかしくなる?????。
        pressure = air_pressure_sensor.readFloatPressure();
        height = (1-pow(pressure/airpressure_on_the_ground,0.19035714))/0.0065*(temperature_on_the_ground+temperature_correction);
        EEPROM.get(4,time_cds);
        now_time = millis();
        if(height <= release_height){
            air_pressure_sensor.setMode(MODE_SLEEP); //気圧センサスリープモード(電源消費を抑える)
            return height;
        }else if((now_time - time_cds) > 400000){ //6分後、勝手に切れる。
            air_pressure_sensor.setMode(MODE_SLEEP); //気圧センサスリープモード(電源消費を抑える)
            return height;
        }else{
            delay(10); //落下速度はそこそこあるので厳しく取る。
            continue;
        }
    }
}

void nichromecut()
{
    digitalWrite(nichrome_pin_1,HIGH);
    time_nichromecut1_start = millis();
    while(true){
        char buf[128];
        LoRa_recv(buf);
        if(strstr(buf,"cutoff")!= NULL){
            digitalWrite(nichrome_pin_1,LOW);
            time_nichromecut2_start = millis();
            delay(100);
            nichromecut2();
            break;
        }
    }
    return;
}

void LoRa_recv(char *buf)
{
    while(true){
        time_nichromecut1_end = millis();
        if((time_nichromecut1_end-time_nichromecut1_start) > 15000){
            strcpy(buf,"cutoff");
            return;
        }
        while(LoRa.available() > 0){
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

void nichromecut2()
{
    digitalWrite(nichrome_pin_2,HIGH);

    EEPROM.write(time_nichromecut1_start,EEPROM_Address);
    EEPROM_Address += 4;
    EEPROM.write(time_nichromecut1_end,EEPROM_Address);
    EEPROM_Address += 4;
    EEPROM.write(time_nichromecut2_start,EEPROM_Address);
    EEPROM_Address += 4;
    
    LoRa_change_destination();

    delay(12000);

    digitalWrite(nichrome_pin_2,LOW);
    delay(100);
    digitalWrite(nichrome_pin_1,HIGH);
    digitalWrite(nichrome_pin_2,HIGH);

    return;
}

void LoRa_change_destination()
{
    LoRa.write("config\r\n");
    delay(10);
    LoRa_reset();
    delay(2000);
    LoRa.write("2\r\n");
    delay(10);
    LoRa.write("g 0\r\n");
    delay(10);
    LoRa.write("q 2\r\n");
    delay(10);
    LoRa.write("save\r\n");
    delay(10);
    LoRa.write("start\r\n");

    return;
}

void LoRa_reset()
{
    digitalWrite(LoRa_rst,LOW);
    delay(1); //1msで十分
    digitalWrite(LoRa_rst,HIGH);
    return;
}

void senttoLora(float height)
{
    LoRa.print("Case has released.\r\n");
    delay(500);
    LoRa.print("The pointed height has arrived.\r\n");
    delay(500);
    LoRa.print("Altitude: ");
    delay(500);
    LoRa.println(height);
    delay(500);
    LoRa.print("The aircraft has released.\r\n");
    delay(500);
    return;
}

void gps_transmission()
{
    while(true){
        while (Serial.available() > 0){
            char c = Serial.read();
            gps.encode(c);
            if(gps.location.isUpdated()){
                LoRa.print("LAT=");delay(100);LoRa.println(gps.location.lat(),10);delay(100);delay(500);
                LoRa.print("LONG=");delay(100);LoRa.println(gps.location.lng(),10);delay(100);delay(500);
                LoRa.print("ALT=");delay(100);LoRa.println(gps.altitude.meters(),10);delay(100);delay(500);
            }
        }
        delay(500);
    }
}
