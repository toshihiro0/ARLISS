/*****************************************
 * PPMによりフライトモードを変更するためのコード
 * Toshihiro Suzuki
 *****************************************/
#define outpin 3

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

  OnePulth(19000-ppmWaitTimeSum);
}

void loop(){
  int ch[8];
  int flightmode1[8]={0,0,0,0,165,0,0,0};
  int flightmode2[8]={0,0,0,0,415,0,0,0};
  int flightmode3[8]={0,0,0,0,815,0,0,0};
  int j;
  
  for(int i=0;i<8;i++){
    ch[i]=flightmode1[i];
  }
  for(j = 0;j < 5;++j){
    ChangeFlightModeTest(ch);
  }
  delay(3000);
  for(int i=0;i<8;i++){
    ch[i]=flightmode2[i];
  }
  for(j = 0;j < 5;++j){
    ChangeFlightModeTest(ch);
  }
  delay(3000);
  for(int i=0;i<8;i++){
    ch[i]=flightmode3[i];
  }
  for(j = 0;j < 5;++j){
    ChangeFlightModeTest(ch);
  }
  delay(3000);
}
