void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.write(8);  // ここの値を変更
  delay(2000);
}
