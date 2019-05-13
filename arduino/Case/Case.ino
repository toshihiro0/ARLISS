#include <SoftwareSerial.h>
#include "Arduino.h"
#include "SPI.h"
#include "AE_MPL115A1.h"

//高度判定部分は実装していません。

SoftwareSerial mySerial(2,3); //RX,TX,GPS通信用

void Cds(void);

void setup()
{
    pinMode(4,OUTPUT);
    analogReference(DEFAULT);
}

void loop()
{
    Cds(); //明暗の判定
    digitalWrite(4,HIGH); //ニクロム線のMOS-FET,ON
    while(true); //4を流しっぱなしにしてニクロム線を切る
}

void Cds()
{
    int i,cds_sum = 0;
    int analogpin = 7;    
    while(true){
        for(i = 0;i < 5;++i){
            cds_sum += analogRead(analogpin); //アナログピンの読み取り
            delay(2); //少しずつ遅らせて取らないと、何回も計測する意味が無い
        }
        int cds_value = cds_sum/5;
        if(cds_sum >= 512){ //暗い時はVoltageが小さい、明るい時はVoltageが大きい
            return; //関数を抜ける
        }else{
            cds_sum = 0; //初期化
            continue; //また計測
        }
    }
}