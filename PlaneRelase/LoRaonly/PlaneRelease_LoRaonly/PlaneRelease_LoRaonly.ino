#include <string.h>

void setup()
{
}

void nichromcut()
{
    digitalWrite(nichrome_pin_1,HIGH);
    delay(3000); //確実にPEラインを溶断するための3秒
    LoRa.print("cutoff\r");
    digitalWrite(nichrome_pin_1,LOW);

    LoRa_recv(); //エレベータ機首上げ状態待ち
    delay(600); //エレベーター機首上げの600ms待ち

    digitalWrite(nichrome_pin_2,HIGH);
    delay(3000); //3秒溶断
    digitalWrite(nichrome_pin_2,LOW);
}

void LoRa_recv(){
    char buf[128];
    char* temp; //開始を記憶
    int time_temp_1 = millis(); //時間による冗長系
    int time_temp_2;
    while (true) {
        temp = buf;
        while (LoRa.available() > 0){
            *temp++ = LoRa.read();
            if (*(temp-1) == '\r') {
                *temp = '\0';
                break;
            }
        }
        time_temp_2 = millis();
        if((time_temp_2 - time_temp_1) > 2000){ //時間による冗長系
            return; //明らかに届くのがおかしいので返れる。
        }else if(strstr(buf, "cutoff") != NULL){
            return; //cutoffが入っていたので帰れる
        }else{
            continue; //まだなのでcutoffが来るまで待つ。
        }
    }
}