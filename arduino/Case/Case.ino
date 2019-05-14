#include <SoftwareSerial.h>
#include <SparkFunBME280.h>

static const int cds_pin = 4; //#defineとstatic const のどちらが良いか僕にはわからない。
static const int SPI_CS_PIN = 10; //気圧センサ

SoftwareSerial GPS(2,3); //RX,TX,GPS通信用
BME280 sensor; 

void cds(void);
void BME280(void);

void setup()
{
    pinMode(cds_pin,OUTPUT);
    analogReference(DEFAULT);
}

void loop()
{
    Cds(); //明暗の判定
    BME280();
    digitalWrite(4,HIGH); //ニクロム線のMOS-FET,ON
    while(true); //4を流しっぱなしにしてニクロム線を切る
}

void cds()
{
    int i,cds_Sum = 0;
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

void BME280()
{

}