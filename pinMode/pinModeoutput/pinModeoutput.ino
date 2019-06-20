void setup()
{
  pinMode(8,INPUT_PULLUP);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
}

void loop()
{
  if(digitalRead(8) == HIGH){
    digitalWrite(3,HIGH);
    digitalWrite(2,LOW);
  }else{
    digitalWrite(2,HIGH);
    digitalWrite(3,LOW);
  }
}
