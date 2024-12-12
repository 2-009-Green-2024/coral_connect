void setup() {
  // put your setup code here, to run once:
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  tone(3, 5000, 1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  tone(3, 5000, 1000);
  delay(1000);
  tone(3, 10000, 1000);
  delay(1000);
}
