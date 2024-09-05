
const int ENA =  3; 
const int ENB = 6;
const int IN1 = 2;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 7;

//irp_1  2  3  4  5  6  7 8
//pin[13,A0,A2,A3,A4,A5,9,8]
int s3 = A2;
int s4 = A3;
int s5 = A4;
int s6 = A5;
int s2 = A0;
int s7 = 9;

const int mspeed =  190;
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

  pinMode(s2,INPUT);
  pinMode(s7,INPUT);
  pinMode(s5,INPUT);
  pinMode(s6,INPUT);

  analogWrite(ENA,mspeed);
  analogWrite(ENB,mspeed);
  delay(600);

}

void loop() {
  // put your main code here, to run repeatedly:
  int  r2 = digitalRead(s2);
  int  r7 = digitalRead(s7);
  int  r8 = digitalRead(8);
  int  r1 = digitalRead(13);
  int  r5 = digitalRead(s5);
  int  r6 = digitalRead(s6);
  int  r4 =  digitalRead(A3);


  if ((r2 == 0) && (r7 == 0)) {
    forward();
    
  }
  if ((r2  == 1) && (r7 == 0)) {
    right();
    delay(100);

  }
  if ((r2  == 0) && (r7 == 1)) {
    left();
    delay(100);
  }
  if ((r5 == 1) && (r4 == 1)){
    forward();
  }
  if (r1 == 1){
    right();
    delay(160);
  }

  if (r8  == 1){
    left();
    delay(160);
  } 
  
  else {
    forward();
  }
}

void forward() {
  analogWrite(ENA, mspeed); 
  analogWrite(ENB, mspeed);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH); 
}

void stop() {
  analogWrite(ENA, 0); 
  analogWrite(ENB, 0); 

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void right() {
  analogWrite(ENA, 250); // Set speed for right motor
  analogWrite(ENB, 180); // Set speed for left motor

  digitalWrite(IN1, LOW);  // Right Motor Backward Pin
  digitalWrite(IN2, HIGH); // Right Motor Forward Pin
  digitalWrite(IN3, LOW);  // Left Motor Backward Pin
  digitalWrite(IN4, HIGH);
  delay(80); // Left Motor Forward Pin
}

void left() {
  analogWrite(ENA, 180); // Set speed for right motor
  analogWrite(ENB, 250); // Set speed for left motor

  digitalWrite(IN1, HIGH); // Right Motor Forward Pin
  digitalWrite(IN2, LOW);  // Right Motor Backward Pin
  digitalWrite(IN3, HIGH); // Left Motor Forward Pin
  digitalWrite(IN4, LOW);
  delay(80);  // Left Motor Backward Pin
}

