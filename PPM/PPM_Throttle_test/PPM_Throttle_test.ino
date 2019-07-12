#define outpin 18

void setup()
{
    pinMode(outpin,OUTPUT);
}

void loop()
{
    int i,j;
    int ch[8] = {500,500,300,500,165,500,500,0};
    for(i = 3;i <= 9;++i){
                ch[2] = i*100;
                for(j = 0;j < 14;++j){
                    PPM_Transmit(ch); //7*14*20 = 1960で2秒間かけてプロペラ回転
                }
            }
    for(i = 0;i < 100;++i){ //2*1000/20 = 100
        PPM_Transmit(ch);
    }
    ch[2] = 0;
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
