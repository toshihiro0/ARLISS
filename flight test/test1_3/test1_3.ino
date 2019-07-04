#define RC_pin 18 //PPM
#define button_pin 14//A0

void setup()
{
  	pinMode(RC_pin,OUTPUT);
    pinMode(button_pin,INPUT_PULLUP);
}

void loop()
{
    int i; //ループの数
    int time1,time2;
    //int PPMMODE_example[8] = {A,E,T,R,C,R,A,?}
    //エレベーター上げるが1900でエレベーター下げるが1100
    //ラダーは知らん
    int PPMMODE_Wait[8] = {500,500,0,500,100,500,500,0};
    int PPMMODE_Arm[8] = {500,500,0,1000,100,1000,500,0}; //アームはラダー900では足りない、1000必要
    int PPMMODE_STABILIZE[8] = {500,500,900,500,425,500,500,0}; //throttle全開
    int PPMMODE_STABILIZE_rudder_left[8] = {500,500,900,100,425,100,500,0}; //左にラダーを切る
    int PPMMODE_STABILIZE_rudder_right[8] = {500,500,900,900,425,900,500,0}; //右にラダーを切る
    int PPMMODE_DEEPSTALL[8] = {500,100,0,500,425,500,500,0};   

    while(true){
        for(i = 0;i < 10;++i){ //Manual確定
            PPM_Transmit(PPMMODE_Wait);
        }
        while(true){
            if(digitalRead(button_pin) == LOW){
                time1 = millis();
                break;
            }else{
                PPM_Transmit(PPMMODE_Wait);
            }
        }
        while(digitalRead(button_pin) == LOW){} //ボタン押されている間ずっとここ
        time2 = millis();
        if((time2-time1) >= 2000){
            continue;
        }else{
            for(i = 0;i < 300;++i){ //6*1000/20 = 300
                PPM_Transmit(PPMMODE_Arm); //Armする。
            }
        }

        while(digitalRead(button_pin) == HIGH){} //アーム後のボタン押され待ち
        time1 = millis();
        while(digitalRead(button_pin) == LOW){} //ボタン押されている間待っている。
        time2 = millis();
        if((time2-time1) >= 2000){
            continue;
        }else{
            for(i = 0;i < 10;++i){ //200ms
                PPM_Transmit(PPMMODE_STABILIZE);
            }
        }

        while(digitalRead(button_pin) == HIGH){}
        time1 = millis();
        while(digitalRead(button_pin) == LOW){} //ボタン押されている間待っている。
        time2 = millis();
        if((time2-time1) >= 2000){
            continue;
        }else{
            break;
        }
    }

    for(i = 0;i < 500;++i){ //10*1000/20 = 500 //10s,stabilize
        PPM_Transmit(PPMMODE_STABILIZE);
    }
    for(i = 0;i < 250;++i){ //5*1000/20 = 250 //5s,auto
        PPM_Transmit(PPMMODE_STABILIZE_rudder_left);
    }
    for(i = 0;i < 250;++i){ //5*1000/20 = 250 //5s,auto
        PPM_Transmit(PPMMODE_STABILIZE_rudder_right);
    }
    while(true){
        PPM_Transmit(PPMMODE_DEEPSTALL); //Deepstall しっぱなし
    }
}

void OnePulth(int PPMtime)
{
    digitalWrite(RC_pin,HIGH);
    delayMicroseconds(250);
    digitalWrite(RC_pin,LOW);
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

    OnePulth(20000-ppmWaitTimeSum);
}
