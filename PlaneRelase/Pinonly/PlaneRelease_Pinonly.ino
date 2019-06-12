#define case_to_plane 18
#define plane_to_case 19

void setup()
{
    pinMode(case_to_plane,OUTPUT);
    digitalWrite(case_to_plane,HIGH);
    pinMode(plane_to_case,INPUT);
}

void nichromcut()
{
    int time_temp_1,time_temp_2;
    digitalWrite(nichrome_pin_1,HIGH);
    delay(3000); //ピンが抜けてしまっていても、確実にPEラインを溶断するための3秒
    while(true){
        if(plane_to_case == LOW){
            digitalWrite(nichrome_pin_1,LOW);
            digitalWrite(case_to_plane,LOW);
            break;
        }else{
            delay(10);
        }
    }
    delay(600); //機首上げ待ち
    digitalWrite(nichrome_pin_2,HIGH);
    delay(3000);
    digitalWrite(nichrome_pin_2,LOW);
}