#define outpin 18

void setup()
{
    pinMode(outpin,OUTPUT);
}

void loop()
{
    /*int ch[8] = {500,500,0,500,165,0,0,0};
    int i,a = 1;
    while(true){
        if(ch[1] == 900 || ch[1] == 100){
          a *= -1;
        }
        ch[1] += a*100;
        for(i = 0;i < 10;++i){
            PPM_Transmit(ch);
        }
    }*/
    int ch[8] = {100,900,0,500,165,500,100,0};
    while(true){
      PPM_Transmit(ch);
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
