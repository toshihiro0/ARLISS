/*****************************************
 * PPMによりフライトモードを変更するためのコード
 * Toshihiro Suzuki
 *****************************************/
#define outpin 13

void setup(){
  Serial.begin(9600);
  pinMode(outpin,OUTPUT);
}

void OnePulth(int PPMtime){
  digitalWrite(outpin,HIGH);
  delayMicroseconds(250);
  digitalWrite(outpin,LOW);
  delayMicroseconds(750+PPMtime);
}

void ChangeFlightModeTest(int ch[8]){
  int ppmWaitTimeSum=0;
 
  for(int i=0;i<8;i++){
    ppmWaitTimeSum += ch[i]+1000;
  }
  
  for(int i=0;i<8;i++){
     OnePulth(ch[i]);
  }

  OnePulth(20000-ppmWaitTimeSum);
}

void loop(){
  int ch[8];
  ch[0]=0;
  ch[1]=0;
  ch[2]=0;
  ch[3]=0;
  ch[4]=100;//CH5に対応
  ch[5]=0;
  ch[6]=0;
  ch[7]=0;
  ChangeFlightModeTest(ch);
}
