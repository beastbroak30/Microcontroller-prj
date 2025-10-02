// Motor driver pins
#define AIN1 5 
#define AIN2 6 
#define BIN1 7 
#define BIN2 8 
#define PWMA 9 
#define PWMB 10 
#define STBY 3
#define CALIB_BUTTON 11   // button for auto-calibration

#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

#define Kp 6
#define Kd 10
#define MaxSpeed 230
#define BaseSpeed 180
#define speedturn 155
#define uturnSpeed 255 // Increased speed for sharper U-turn
#define turnDuration   100 // Duration for each turn in milliseconds when all white

int lastError = 0;

void setup()
{   
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);
    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(3000);
    for (int i = 0; i < 100; i++)
    {   
        if (i < 25 || i >= 75) 
        {   
            move(0, speedturn, 1);
            move(1, speedturn, 0);
        }
        else
        {   
            move(0, speedturn, 0);
            move(1, speedturn, 1);
        }
        qtr.calibrate();
        delay(20);
    }
    delay(3000); 
}  

void loop()
{   
    uint16_t position = qtr.readLineBlack(sensorValues);
    
    // Check if all sensors detect white (values typically < 200 for white)
    bool allWhite = true;
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        if (sensorValues[i] > 200) // Adjust threshold if needed
        {
            allWhite = false;
            break;
        }
    }
    
    if (allWhite)
    {
        // Perform right turn
        move(0, speedturn, 0); // Left motor backward
        move(1, speedturn, 1); // Right motor forward
        delay(turnDuration);
        // Perform first left turn
        move(0, speedturn, 1); // Left motor forward
        move(1, speedturn, 0); // Right motor backward
        delay(turnDuration);
        // Perform second left turn
        move(0, speedturn, 1); // Left motor forward
        move(1, speedturn, 0); // Right motor backward
        delay(turnDuration);
        // Stop motors and blink LED
        while (allWhite)
        {
            move(0, 0, 0); // Stop left motor
            move(1, 0, 0); // Stop right motor
            digitalWrite(LED_BUILTIN, HIGH);
            delay(250); // LED on for 250ms
            digitalWrite(LED_BUILTIN, LOW);
            delay(250); // LED off for 250ms
            qtr.readLineBlack(sensorValues);
            allWhite = true;
            for (uint8_t i = 0; i < SensorCount; i++)
            {
                if (sensorValues[i] > 200) // Detect black
                {
                    allWhite = false;
                    break;
                }
            }
        }
        return;
    }
    
    // Check if all sensors detect black (values typically > 800 for black)
    bool allBlack = true;
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        if (sensorValues[i] < 600) // Adjust threshold if needed
        {
            allBlack = false;
            break;
        }
    }
    
    if (allBlack)
    {
        // Perform sharper U-turn with higher speed and opposite directions
        while (allBlack)
        {
            move(0, uturnSpeed, 1); // Left motor forward at higher speed
            move(1, uturnSpeed, 0); // Right motor backward at higher speed
            qtr.readLineBlack(sensorValues);
            allBlack = true;
            for (uint8_t i = 0; i < SensorCount; i++)
            {
                if (sensorValues[i] < 600) // Detect white
                {
                    allBlack = false;
                    break;
                }
            }
            delay(10); // Reduced delay for faster response
        }
        return;
    }
    
    if (position > 6700)
    {   
        move(1, speedturn, 1);
        move(0, speedturn, 0);
        return;    
    }
    if (position < 300)
    {   
        move(1, speedturn, 0);
        move(0, speedturn, 1);
        return;
    }
    
    int error = position - 3500;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed - motorSpeed;
  
    if (rightMotorSpeed > MaxSpeed) rightMotorSpeed = MaxSpeed; 
    if (leftMotorSpeed > MaxSpeed) leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;    
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;
    
    move(1, rightMotorSpeed, 1);
    move(0, leftMotorSpeed, 1);
}

void move(int motor, int speed, int direction)
{   
    boolean inPin1 = HIGH, inPin2 = LOW;
    if (direction == 1)
    {
        inPin1 = HIGH;
        inPin2 = LOW;
    }  
    if (direction == 0)
    {
        inPin1 = LOW;
        inPin2 = HIGH;
    }
    if (motor == 0)
    {   
        digitalWrite(AIN1, inPin1);
        digitalWrite(AIN2, inPin2);
        analogWrite(PWMA, speed);
    }
    if (motor == 1)
    {   
        digitalWrite(BIN1, inPin1);
        digitalWrite(BIN2, inPin2);
        analogWrite(PWMB, speed);
    }  
}