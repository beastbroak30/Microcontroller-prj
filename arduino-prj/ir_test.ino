int ir8 = 8;

void setup() {
  // put your setup code here, to run once:
  pinMode(ir8,INPUT);
  Serial.begin(115200);
  Serial.println('ir test_1');

}

void loop() {
  // put your main code here, to run repeatedly:
  int ir = digitalRead(ir8);

  Serial.println(ir);
  delay(500);

}
