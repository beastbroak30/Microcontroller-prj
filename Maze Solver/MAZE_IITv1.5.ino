#include <QTRSensors.h>
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Tunable parameters
#define Kp 6
#define Kd 10
#define MaxSpeed 230
#define BaseSpeed 200
#define speedturn 155
#define uturnSpeed 255
#define fastUturnSpeed 255
#define turnDuration 100
#define WHITE_CONFIRM_ATTEMPTS 1
#define BLACK_CONFIRM_DELAY 100
#define NODE_THRESHOLD 500
#define MAX_NODES 20
#define CONSECUTIVE_TURN_LIMIT 3

// ADD YOUR DESIRED CALIBRATION SPEED HERE
#define calibrationSpeed 150    // <<<<< CHANGE THIS VALUE FOR CALIBRATION SPEED

// Motor driver pins
#define AIN1 5 
#define AIN2 6 
#define BIN1 7 
#define BIN2 8 
#define PWMA 9 
#define PWMB 10 
#define STBY 3

// Buttons
#define START_DRY_RUN_BUTTON 12   // New: Start dry run after calibration
#define START_FINAL_RUN_BUTTON 11 // Original: Start final run after shortest path

// Direction indices
#define NORTH 1
#define EAST  2
#define SOUTH 3
#define WEST  4

int lastError = 0;
bool isDryRun = true;
bool isStopped = false;
bool isActualRun = false;
int dir_arr[MAX_NODES] = {0};
int nodeCount = 0;
unsigned long lastNodeTime = 0;
bool isReturning = false;
int currentDirection = NORTH;
int consecutiveLeftTurns = 0;
int consecutiveRightTurns = 0;

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
    
    // Buttons with internal pull-up
    pinMode(START_DRY_RUN_BUTTON, INPUT_PULLUP);
    pinMode(START_FINAL_RUN_BUTTON, INPUT_PULLUP);
    
    digitalWrite(STBY, HIGH);
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);  // LED ON during calibration
    
    delay(5000);

    // === CALIBRATION WITH CUSTOM SPEED ===
    for (int i = 0; i < 100; i++)
    {   
        if (i < 25 || i >= 75) 
        {   
            move(0, calibrationSpeed, 1);  // Left motor forward
            move(1, calibrationSpeed, 0);  // Right motor backward
        }
        else
        {   
            move(0, calibrationSpeed, 0);  // Left motor backward
            move(1, calibrationSpeed, 1);  // Right motor forward
        }
        qtr.calibrate();
        delay(20);
    }
    
    move(0, 0, 0);  // Stop motors after calibration
    move(1, 0, 0);
    
    digitalWrite(LED_BUILTIN, LOW);
    
    // === WAIT FOR D12 BUTTON TO START DRY RUN ===
    while (digitalRead(START_DRY_RUN_BUTTON) == HIGH) 
    {
        delay(50);  // Wait until button on D12 is pressed
    }
    delay(300); // Debounce
    
    lastNodeTime = millis();
    if (nodeCount < MAX_NODES)
    {
        dir_arr[nodeCount++] = NORTH;
    }
}

void loop()
{   
    // === START FINAL RUN WITH D11 BUTTON (D11) ===
    if (digitalRead(START_FINAL_RUN_BUTTON) == LOW)
    {
        delay(50); // Debounce
        if (digitalRead(START_FINAL_RUN_BUTTON) == LOW)
        {
            if (isDryRun)
            {
                isDryRun = false;
                isStopped = true;
                move(0, 0, 0);
                move(1, 0, 0);
                digitalWrite(LED_BUILTIN, HIGH);
                processShortestPath();
                while (digitalRead(START_FINAL_RUN_BUTTON) == LOW);
                delay(50);
                return;
            }
            else if (isStopped)
            {
                isStopped = false;
                isActualRun = true;
                digitalWrite(LED_BUILTIN, LOW);
                while (digitalRead(START_FINAL_RUN_BUTTON) == LOW);
                delay(50);
            }
        }
    }

    if (isStopped)
    {
        move(0, 0, 0);
        move(1, 0, 0);
        return;
    }

    uint16_t position = qtr.readLineBlack(sensorValues);
    
    // === ALL YOUR ORIGINAL LOGIC BELOW (UNCHANGED) ===
    
    bool allWhite = true;
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        if (sensorValues[i] > 200)
        {
            allWhite = false;
            break;
        }
    }
    
    if (allWhite)
    {
        int confirmCount = 0;
        for (int attempt = 0; attempt < WHITE_CONFIRM_ATTEMPTS; attempt++)
        {
            move(0, speedturn, 0);
            move(1, speedturn, 1);
            delay(turnDuration);
            move(0, speedturn, 1);
            move(1, speedturn, 0);
            delay(turnDuration);
            move(0, speedturn, 1);
            move(1, speedturn, 0);
            delay(turnDuration);
            qtr.readLineBlack(sensorValues);
            allWhite = true;
            for (uint8_t i = 0; i < SensorCount; i++)
            {
                if (sensorValues[i] > 200)
                {
                    allWhite = false;
                    break;
                }
            }
            if (!allWhite) break;
            confirmCount++;
        }
        if (confirmCount == WHITE_CONFIRM_ATTEMPTS)
        {
            while (allWhite)
            {
                move(0, 0, 0);
                move(1, 0, 0);
                digitalWrite(LED_BUILTIN, HIGH);
                delay(250);
                digitalWrite(LED_BUILTIN, LOW);
                delay(250);
                qtr.readLineBlack(sensorValues);
                allWhite = true;
                for (uint8_t i = 0; i < SensorCount; i++)
                {
                    if (sensorValues[i] > 200)
                    {
                        allWhite = false;
                        break;
                    }
                }
            }
            if (isDryRun)
            {
                isDryRun = false;
                isStopped = true;
                digitalWrite(LED_BUILTIN, HIGH);
                processShortestPath();
            }
            else if (isActualRun && !isReturning)
            {
                isReturning = true;
                reversePath();
            }
            return;
        }
        return;
    }
    
    bool allBlack = true;
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        if (sensorValues[i] < qtr.calibrationOn.minimum[i] + 5)
        {
            allBlack = false;
            break;
        }
    }
    
    if (allBlack)
    {
        unsigned long startTime = millis();
        while (millis() - startTime < BLACK_CONFIRM_DELAY)
        {
            qtr.readLineBlack(sensorValues);
            allBlack = true;
            for (uint8_t i = 0; i < SensorCount; i++)
            {
                if (sensorValues[i] < qtr.calibrationOn.minimum[i] + 5)
                {
                    allBlack = false;
                    break;
                }
            }
            if (!allBlack) return;
            delay(10);
        }
        if (isDryRun && nodeCount < MAX_NODES)
        {
            currentDirection = (currentDirection - 2 + 4) % 4;
            if (currentDirection == 0) currentDirection = 4;
            dir_arr[nodeCount++] = currentDirection;
            lastNodeTime = millis();
            consecutiveLeftTurns = 0;
            consecutiveRightTurns = 0;
        }
        while (allBlack)
        {
            move(0, fastUturnSpeed, 1);
            move(1, fastUturnSpeed, 0);
            qtr.readLineBlack(sensorValues);
            allBlack = true;
            for (uint8_t i = 0; i < SensorCount; i++)
            {
                if (sensorValues[i] < qtr.calibrationOn.minimum[i] + 5)
                {
                    allBlack = false;
                    break;
                }
            }
            delay(10);
        }
        return;
    }
    
    char movement = 'S';
    bool inverseTurn = false;
    if (position > 6700)
    {
        consecutiveRightTurns++;
        consecutiveLeftTurns = 0;
        if (isDryRun && consecutiveRightTurns >= CONSECUTIVE_TURN_LIMIT)
        {
            movement = 'L';
            inverseTurn = true;
            consecutiveRightTurns = 0;
            move(1, speedturn, 0);
            move(0, speedturn, 1);
        }
        else
        {
            movement = 'R';
            move(1, speedturn, 1);
            move(0, speedturn, 0);
        }
    }
    else if (position < 300)
    {
        consecutiveLeftTurns++;
        consecutiveRightTurns = 0;
        if (isDryRun && consecutiveLeftTurns >= CONSECUTIVE_TURN_LIMIT)
        {
            movement = 'R';
            inverseTurn = true;
            consecutiveLeftTurns = 0;
            move(1, speedturn, 1);
            move(0, speedturn, 0);
        }
        else
        {
            movement = 'L';
            move(1, speedturn, 0);
            move(0, speedturn, 1);
        }
    }
    else
    {
        consecutiveLeftTurns = 0;
        consecutiveRightTurns = 0;
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

    if (isDryRun && nodeCount < MAX_NODES)
    {
        if (movement == 'L' || movement == 'R')
        {
            if (inverseTurn)
            {
                currentDirection = (currentDirection + 1) % 4;
                if (currentDirection == 0) currentDirection = 4;
            }
            else
            {
                currentDirection = (currentDirection - 1 + 4) % 4;
                if (currentDirection == 0) currentDirection = 4;
            }
            dir_arr[nodeCount++] = currentDirection;
            lastNodeTime = millis();
        }
        else if (movement == 'S' && (millis() - lastNodeTime >= NODE_THRESHOLD))
        {
            currentDirection = (currentDirection - 1 + 4) % 4;
            if (currentDirection == 0) currentDirection = 4;
            dir_arr[nodeCount++] = currentDirection;
            lastNodeTime = millis();
        }
    }
    else if (isActualRun && nodeCount > 0)
    {
        static int pathIndex = 0;
        int expectedDir = dir_arr[isReturning ? (nodeCount - 1 - pathIndex % nodeCount) : (pathIndex % nodeCount)];
        
        bool match = false;
        if ((expectedDir == NORTH && position >= 300 && position <= 6700) ||
            (expectedDir == EAST && position > 6700) ||
            (expectedDir == SOUTH && position >= 300 && position <= 6700) ||
            (expectedDir == WEST && position < 300) ||
            (expectedDir == (isReturning ? ((currentDirection + 1) % 4 == 0 ? 4 : (currentDirection + 1) % 4) : currentDirection) && allBlack))
        {
            match = true;
            if (expectedDir == (isReturning ? ((currentDirection + 1) % 4 == 0 ? 4 : (currentDirection + 1) % 4) : currentDirection) && allBlack)
            {
                unsigned long startTime = millis();
                while (millis() - startTime < BLACK_CONFIRM_DELAY)
                {
                    qtr.readLineBlack(sensorValues);
                    allBlack = true;
                    for (uint8_t i = 0; i < SensorCount; i++)
                    {
                        if (sensorValues[i] < qtr.calibrationOn.minimum[i] + 5)
                        {
                            allBlack = false;
                            break;
                        }
                    }
                    if (!allBlack) break;
                    delay(10);
                }
                if (allBlack)
                {
                    while (allBlack)
                    {
                        move(0, fastUturnSpeed, 1);
                        move(1, fastUturnSpeed, 0);
                        qtr.readLineBlack(sensorValues);
                        allBlack = true;
                        for (uint8_t i = 0; i < SensorCount; i++)
                        {
                            if (sensorValues[i] < qtr.calibrationOn.minimum[i] + 5)
                            {
                                allBlack = false;
                                break;
                            }
                        }
                        delay(10);
                    }
                    currentDirection = (currentDirection - 2 + 4) % 4;
                    if (currentDirection == 0) currentDirection = 4;
                }
            }
            else
            {
                currentDirection = expectedDir;
            }
            pathIndex++;
        }
    }
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

void processShortestPath()
{
    bool changed = true;
    while (changed && nodeCount > 1)
    {
        changed = false;
        int newCount = 0;
        int temp[MAX_NODES] = {0};
        temp[0] = dir_arr[0];
        newCount = 1;
        
        for (int i = 1; i < nodeCount; i++)
        {
            if (newCount > 0 && 
                ((temp[newCount-1] == NORTH && dir_arr[i] == SOUTH) ||
                 (temp[newCount-1] == SOUTH && dir_arr[i] == NORTH) ||
                 (temp[newCount-1] == EAST && dir_arr[i] == WEST) ||
                 (temp[newCount-1] == WEST && dir_arr[i] == EAST)))
            {
                newCount--;
                changed = true;
            }
            else if (newCount < MAX_NODES)
            {
                temp[newCount++] = dir_arr[i];
            }
        }
        
        for (int i = 0; i < newCount; i++)
        {
            dir_arr[i] = temp[i];
        }
        nodeCount = newCount;
    }
}

void reversePath()
{
    for (int i = 0; i < nodeCount / 2; i++)
    {
        int temp = dir_arr[i];
        dir_arr[i] = dir_arr[nodeCount - 1 - i];
        dir_arr[nodeCount - 1 - i] = temp;
    }
    for (int i = 0; i < nodeCount; i++)
    {
        if (dir_arr[i] == NORTH) dir_arr[i] = SOUTH;
        else if (dir_arr[i] == SOUTH) dir_arr[i] = NORTH;
        else if (dir_arr[i] == EAST) dir_arr[i] = WEST;
        else if (dir_arr[i] == WEST) dir_arr[i] = EAST;
    }
}