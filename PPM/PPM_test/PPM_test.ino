#define outpin 3

int PPM[8]={500,500,0,500,100,500,500,0};

void setup()
{
    Serial.begin(57600);
}

void loop()
{
    int i,j;
    for(i = 1;i < 10;++i){
        PPM[4] = i*100;
        Serial.println(i*100);
        for(j = 0;j < 200;++j){
            PPM_Transmit(PPM);
        }
    }
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
