#define outpin 3
int PPMMODE_Arm[8] = {500,500,0,1000,100,1000,500,0}; //アームはラダー900では足りない、1000必要

void setup()
{
    pinMode(outpin,OUTPUT);
    delay(5000);
}

void loop()
{
    int i;
    for(i = 0;i < 250;++i){
        PPM_Transmit(PPMMODE_Arm);
    }
    while(true){}
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
