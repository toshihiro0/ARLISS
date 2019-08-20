#include <mavlink.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <math.h>

#define outpin 3 //PPM
#define deploy_judge_pin_INPUT1  10 //一段階目溶断の抜けピン
#define deploy_judge_pin_INPUT2  9 //二段階目溶断の抜けピン
#define LoRa_sw 6 //MOSFETのスイッチピン
#define LoRa_rst 18 //LoRaのリセット
#define LoRa_RX 17
#define LoRa_TX 19

#define SLEEP 0
#define TRAINING 1
#define STABILIZE_NOSEUP 2
#define STABILIZE 3
#define AUTO 4
#define DEEPSTALL 5

#define goal_latitude 35.7417229
#define goal_longtitude 140.0101197
#define goal_altitude 45.0
#define difference_lat 111316.2056

static const float difference_lon = cos(goal_latitude/180*M_PI)*M_PI*6378.137/180*1000;

SoftwareSerial LoRa(LoRa_RX,LoRa_TX); //LoRaと接続、PixhawkはSerialでつなぐ。

int EEPROM_Address = 1;
unsigned long int time_auto_zero = 0;//オートが始まった最初の時刻を格納
unsigned long int time_auto = 0;//オートが始まってからの経過時間を格納
int LoRa_send_Mode = 0; //LoRaでどれを送るか決める。

int PPMMODE_Arm[8] = {500,500,0,1000,100,1000,500,0}; //アームはラダー900では足りない、1000必要
int PPMMODE_MANUAL[8] = {500,500,0,500,100,500,500,0}; //ケースに入ってる間
int PPMMODE_TRAINING[8] = {500,500,0,500,300,500,500,0}; //ピン抜け1回後に入る。
int PPMMODE_STABILIZE_NOSEUP[8] = {500,900,0,500,425,500,500,0}; //900側が機首上げ
int PPMMODE_STABILIZE[8] = {500,500,300,500,425,500,500,0}; //300から徐々に上げる
int PPMMODE_AUTO[8] = {500,500,0,500,815,500,500,0};
int PPMMODE_DEEPSTALL[8] = {500,900,0,500,425,500,500,0}; //900側がエレベーター上げ

void setup()
{
    pinMode(outpin,OUTPUT);
    while(true){
        PPM_Transmit(PPMMODE_MANUAL);
    }
}

void OnePulth(int PPMtime)
{
    digitalWrite(outpin,HIGH);
    delayMicroseconds(250);
    digitalWrite(outpin,LOW);
    delayMicroseconds(750+PPMtime);
}

void PPM_Transmit(int ch[8])
{
    int ppmWaitTimeSum=0;

    for(int i=0;i<8;i++){
        ppmWaitTimeSum += ch[i]+1000;
    }

    for(int i=0;i<8;i++){
        OnePulth(ch[i]);
    }

    OnePulth(19000-ppmWaitTimeSum);
}
