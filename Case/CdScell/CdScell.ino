int analogpin0 = 0; //アナログピン選択

int i;
int num_of_loop = 5;
int delay_of_loop = 2;
int sum;

double data;

long msec;
long sec;
long minute;

double vcc = 5.0;
int adcmax = 1024;
double refres = 10.0; // キロオーム
double volt;
double outres; 
double lx;

int INTERVAL = 100;

void setup(){
  analogReference(DEFAULT); // DEFAULT = 5.0 V
  Serial.begin(9600); // Serial Speed
  Serial.print("-------------------\n");
  Serial.print("  CdS Data Logger  \n");
  Serial.print("-------------------\n");
  Serial.print("INTERVAL=");
  Serial.print(INTERVAL);  
  Serial.print("\n");
}

void loop(){
  sum = 0;
  for(i=0; i<num_of_loop; i++){
    sum = sum + analogRead(analogpin0); // read analog0 pin 1024 ch
    delay(delay_of_loop);//delayを使っている間は計算なども止まるため、困るようならその他の方法も考えたほうがいいかも
  }
  data = double(sum) / double(num_of_loop);
  
  sec = msec * 0.001;
  minute = sec/60.0;
  volt = double(data) / double(adcmax) * vcc ; // AD変換で得た電圧値
  outres = refres * (vcc / volt - 1.0) ; //CdSセル自体の抵抗値、回路図を書けば導出可能
  lx = pow(outres/7.0, -1.49); //データシートの特性より
  
  Serial.print(sec);
  Serial.print("sec ");
  Serial.print(minute);
  Serial.print("min ");
  Serial.print(data); 
  Serial.print("ch ");
  Serial.print(volt);
  Serial.print("v ");
  Serial.print(outres);
  Serial.print("kOhm ");
  Serial.print(lx);
  Serial.print("lx \n");
 
  delay(INTERVAL);
  msec = msec + INTERVAL;  
}
