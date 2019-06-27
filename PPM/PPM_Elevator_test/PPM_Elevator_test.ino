#define outpin 13

void setup()
{
    pinMode(outpin,OUTPUT);
}

void loop()
{
    int ch[8] = {500,100,0,500,165,0,0,0}; //下限が1100(こっちが機首上げ),中立が1500,上限1900
    int i,a = 1;
    while(true){
        for(i = 0;i < 10;++i){
            PPM_Transmit(ch);
        }
    }
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

void OnePulth(int PPMtime)
{
    digitalWrite(outpin,HIGH);
    delayMicroseconds(250);
    digitalWrite(outpin,LOW);
    delayMicroseconds(750+PPMtime);
}
