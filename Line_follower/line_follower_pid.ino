

#define ENA 6   // PWM for Motor A speed
#define ENB 5   // PWM for Motor B speed
#define IN1 10  // Motor A direction 1
#define IN2 9   // Motor A direction 2
#define IN3 8   // Motor B direction 1
#define IN4 7 
const int motorspeed =  185;
const int error = 100;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);


  analogWrite(ENA,motorspeed);
  analogWrite(ENB,motorspeed);
  delay(600);

}
int readA(int analogPin) {
  int value = analogRead(analogPin);
  if (value < 80) return 0;
  if (value > 200) return 1;
  return 0;
}
void loop() {
  // put your main code here, to run repeatedly:
  int r1 = readA(A0);
  int r2 = readA(A1);
  int r3 = readA(A2);
  int r4 = readA(A3);
  int r5 = readA(A4);
  int r6 = readA(A5);
  int r7 = readA(A6);
  int r8 = readA(A7);


  if ((r2 == 0) && (r7 == 0)) {
    forward(170);
    
  }
  if ((r2  == 1) && (r7 == 0)) {
    right(255,200);
    delay(120);

  }
  if ((r3 == 1) && (r6 == 0)){
    right(255,225);
    delay(100);
  }
  if ((r3 == 0) && (r6 == 1)){
    left(255);
    delay(100);
  } 
  if ((r2  == 0) && (r7 == 1)) {
    left(255);
    delay(110);
  }
  if (r1 == 1){
    right(255,255);
    delay(120);
  }
  if (((r5 == 1) && (r6 == 1) && (r7 == 1)) || ((r8 == 1) && (r7 == 1) )){
    left(250);
    delay(60);
  }
  if (((r4 == 1) && (r3 == 1) && (r2 == 1)) || ((r2 == 1) && (r1 == 1) )){
    right(250,250);
    delay(60);
  }
  if (r8  == 1){
    left(255);
    delay(100);
  }
  if ((r8  == 1) && (r7 == 1) && (r6 == 1) && (r5  == 1) && (r5  == 1) && (r4 == 1) && (r3 == 1)){
    left(255);
    delay(200);
  } 
  if (r7 == 1){
    left(250);
    delay(100);
  } 
  if (r6 == 1){
    left(250);
    delay(100);
  }
  else {
    forward(180);
  }
}

void forward(int motorSpeed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void backward(int motorSpeed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void right(int motorSpeed,int mEA) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, mEA);
  analogWrite(ENB, motorSpeed);
}

void left(int motorSpeed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, motorSpeed);
  analogWrite(ENB, motorSpeed);
}

void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}