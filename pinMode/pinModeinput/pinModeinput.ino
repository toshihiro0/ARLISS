void setup()
{
  pinMode(4,OUTPUT);
  digitalWrite(4,LOW);
  pinMode(5,OUTPUT);
  digitalWrite(4,HIGH);
  pinMode(12,INPUT_PULLUP); 
}

void loop()
{
  if(digitalRead(12) == LOW){
    digitalWrite(4,HIGH);
    digitalWrite(5,HIGH);
  }else{
    digitalWrite(4,LOW);
    digitalWrite(5,LOW);
  }
}
