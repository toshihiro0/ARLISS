#include <string.h>
#include <SoftwareSerial.h>

#define case_to_plane 18
#define plane_to_case 19
#define nichrome_pin_1 1
#define nichrome_pin_2 2

SoftwareSerial LoRa(8,9);

void setup()
{
    pinMode(case_to_plane,OUTPUT);
    digitalWrite(case_to_plane,HIGH);
    pinMode(plane_to_case,INPUT);
    LoRa.begin(19200);
}

void loop()
{
  
}

void nichromcut()
{
    long time_temp_1;
    boolean timeout;
    digitalWrite(nichrome_pin_1,HIGH);
    delay(3000); //確実にPEラインを溶断するための3秒

    while(true){
        if(plane_to_case == LOW){
            digitalWrite(nichrome_pin_1,LOW);
            digitalWrite(case_to_plane,LOW);
            break;
        }else{
            delay(10);
        }
    }
    time_temp_1 = millis();
    while(true){
        char buf[128];
        timeout = LoRa_or_Pin_recv(buf,time_temp_1); //機首展開待ち
        if(timeout){
            break;
        }else{
            if(strstr(buf,"cutoff")!= NULL){
                break;
            }
        }
    }

    digitalWrite(nichrome_pin_2,HIGH);
    delay(6000); //3秒溶断 //6sの方が良い。
    digitalWrite(nichrome_pin_2,LOW);
}

boolean LoRa_or_Pin_recv(char *buf,long time_temp_1)
{
    long time_temp_2;
    while (true){
        while(LoRa.available() > 0){
            *buf++ = LoRa.read();
            if(*(buf-3) == 'O' && *(buf-2) == 'K' && *(buf-1) == '\r'){
                continue;
            }else if(*(buf-1) == '\r'){
                *buf = '\0';
                return false;
            }
        }
        time_temp_2 = millis();
        if((time_temp_2 - time_temp_1) > 10000){ //時間による冗長系,10000s経って来なかったら仕方ない。
            return true; //明らかに届くのがおかしいので返れる。タイムアウトしたことを知らせる。
        }else{
            continue; //まだなのでcutoffが来るまで待つ。
        }
    }
}
