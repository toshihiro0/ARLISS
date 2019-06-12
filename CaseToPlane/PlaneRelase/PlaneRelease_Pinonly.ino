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
    time_temp_1 = millis();
    while(true){
        if(plane_to_case == LOW){
            digitalWrite(nichrome_pin_1,LOW);
            digitalWrite(case_to_plane,LOW);
            time_temp_2 = millis();
            break;
        }else{
            delay(10);
        }
    }
    delay(5000); //モーター回るの待ち
    digitalWrite(nichrome_pin_2,HIGH);
    delay(3000);
    digitalWrite(nichrome_pin_2,LOW);
}