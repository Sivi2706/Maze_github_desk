#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <EEPROM.h>

// Ultrasonic Sensor Pins
#define FRONT_TRIGGER_PIN 7
#define FRONT_ECHO_PIN 8
#define LEFT_TRIGGER_PIN A3
#define LEFT_ECHO_PIN A2
#define RIGHT_TRIGGER_PIN A0
#define RIGHT_ECHO_PIN A1

// Encoder pin definitions
#define LEFT_ENCODER_PIN 13
#define RIGHT_ENCODER_PIN 12

// Motor Pins
#define IN1 2
#define IN2 4
#define IN3 5
#define IN4 6
#define FNA 3
#define FNB 11

// Target distance to travel (in centimeters)
#define TARGET_DISTANCE 25.0

// Motor calibration to fix right-side drift
#define LEFT_MOTOR_CALIBRATION 1.0
#define RIGHT_MOTOR_CALIBRATION 0.95

// Encoder and distance calculation variables
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;
const unsigned int PULSES_PER_TURN = 20;
const float WHEEL_DIAMETER = 4;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Distance tracking
float leftTotalDistance = 0.0;
float rightTotalDistance = 0.0;
bool isMovingForward = false;
bool targetReached = false;

// Bearing system variables
#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270
int currentBearing = NORTH;
bool correctionEnabled = true;

// Correction timing variables
unsigned long lastCorrectionTime = 0;
const unsigned long CORRECTION_INTERVAL = 500;

// Interrupt service routines
void leftEncoderISR() { leftPulses++; }
void rightEncoderISR() { rightPulses++; }

bool hasStarted = false;
float initialDistance = 0;
float totalDistance = 0;

// RPM calculation variables
unsigned long lastLeftPulseTime = 0;
unsigned long lastRightPulseTime = 0;
float leftRPM = 0.0;
float rightRPM = 0.0;

// ROWS & COLS definitions
#define ROWS 8
#define COLS 8
#define SIZE ROWS * COLS

char movement_arr[SIZE];
int junction_nodes[SIZE];
int junction_visited[SIZE];
int index = 0;
int count = 0;

void calculateRPM() {
    unsigned long currentTime = millis();
    if (leftPulses > 0) {
        leftRPM = (leftPulses / (float)PULSES_PER_TURN) * (60000.0 / (currentTime - lastLeftPulseTime));
        lastLeftPulseTime = currentTime;
        leftPulses = 0;
    }
    if (rightPulses > 0) {
        rightRPM = (rightPulses / (float)PULSES_PER_TURN) * (60000.0 / (currentTime - lastRightPulseTime));
        lastRightPulseTime = currentTime;
        rightPulses = 0;
    }
}

void ultrasonicSetup(int trigPin, int echoPin) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long duration = pulseIn(echoPin, HIGH, 30000);
    return (duration == 0) ? 0 : (duration * 0.0343 / 2.0);
}

int checkDist(int trigPin, int echoPin)
{
    // this function should 
    // return 0 if there is no space in that direction
    // return 1 if there is space in that direction
    // return -1 if the space is more than 200 cm
    float distance = getDistance(trigPin, echoPin);

    Serial.print("Distance: "); Serial.println(distance);

    if (distance > 200) return -1;
    else if (distance <= 10) return 0;
    else return 1;

    Serial.println("Distance checked");
}

// MPU6050 Setup
const int MPU = 0x68;
const float GYRO_SCALE = 1.0 / 131.0;
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;
float accelAngleX, accelAngleY;
float roll, pitch, yaw;
float GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, previousTime, currentTime;
float initialYaw = 0.0;
const float ALPHA = 0.96;

void getOrientation();
void getAcceleration();
void calculateError();
void updateMPU();
float normalizeYaw(float rawYaw);
int getCurrentAbsoluteBearing();
void stopMotors();
void alignToBearing(int targetBearing);
void maintainBearing();
void updateDistance(); 

void getOrientation() {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    gyroX = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    gyroY = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    gyroZ = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
}

void getAcceleration() {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    accelX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

void calculateError() {
    for (int i = 0; i < 1000; i++) {
        getOrientation();
        GyroErrorX += gyroX;
        GyroErrorY += gyroY;
        GyroErrorZ += gyroZ;
        delay(1);
    }
    GyroErrorX /= 1000;
    GyroErrorY /= 1000;
    GyroErrorZ /= 1000;
    Serial.println("Gyroscope calibration complete.");
}

void updateMPU() {
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) * 0.001;
    
    getOrientation();
    getAcceleration();
    
    gyroX -= GyroErrorX;
    gyroY -= GyroErrorY;
    gyroZ -= GyroErrorZ;
    
    accelAngleX = atan2(accelY, sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI;
    accelAngleY = atan2(-accelX, sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI;
    
    roll = ALPHA * (roll + gyroX * elapsedTime) + (1 - ALPHA) * accelAngleX;
    pitch = ALPHA * (pitch + gyroY * elapsedTime) + (1 - ALPHA) * accelAngleY;
    yaw += gyroZ * elapsedTime;
}

float normalizeYaw(float rawYaw) {
    float normalized = fmod(rawYaw, 360.0);
    if (normalized < 0) normalized += 360.0;
    return normalized;
}

int getCurrentAbsoluteBearing() {
    float relativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(relativeYaw);
    int absoluteBearing = round(normalizedYaw / 90) * 90;
    if (absoluteBearing >= 360) absoluteBearing = 0;
    return absoluteBearing;
}

void stopMotors() {
    isMovingForward = false;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stopped.");
}

void alignToBearing(int targetBearing) {
    Serial.println("Aligning...");
    float currentRelativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(currentRelativeYaw);
    
    float error = targetBearing - normalizedYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    const float BEARING_TOLERANCE = 2.0;
    
    if (abs(error) > BEARING_TOLERANCE) {
        if (error > 0) {
            analogWrite(FNA, 80 * LEFT_MOTOR_CALIBRATION);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(FNB, 80 * RIGHT_MOTOR_CALIBRATION);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            Serial.println("Turning LEFT to align");
        } else {
            analogWrite(FNA, 80 * LEFT_MOTOR_CALIBRATION);
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(FNB, 80 * RIGHT_MOTOR_CALIBRATION);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            Serial.println("Turning RIGHT to align");
        }
        
        float currentError;
        do {
            updateMPU();
            currentRelativeYaw = yaw - initialYaw;
            normalizedYaw = normalizeYaw(currentRelativeYaw);
            currentError = targetBearing - normalizedYaw;
            if (currentError > 180) currentError -= 360;
            if (currentError < -180) currentError += 360;
            Serial.print("Aligning - Current yaw: ");
            Serial.print(normalizedYaw);
            Serial.print("°, Error: ");
            Serial.println(currentError);
            delay(10);
        } while (abs(currentError) > BEARING_TOLERANCE * 0.5);
        
        stopMotors();
        Serial.println("Alignment complete");
        currentBearing = targetBearing;
    }
}

void move_forward(int PWM)
{
    Serial.println("Moving forward.");

    // for (int i = 0; i < 5; i++)
    // {
    //     Serial.println(i);
    // }

    // Reset distances and flag for the new movement segment
    leftTotalDistance = 0.0;
    rightTotalDistance = 0.0;
    targetReached = false;
    
    isMovingForward = true;
    analogWrite(FNA, PWM * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, PWM * RIGHT_MOTOR_CALIBRATION);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    //while (isMovingForward && !targetReached) {
    updateMPU();
    float correction = initialYaw - yaw;
    
    if (correction > 3) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else if (correction < -3) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    } else {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    }
    
    updateDistance();
    
    float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;
    if (avgDistance >= TARGET_DISTANCE) {
        stopMotors();
        targetReached = true;
        Serial.println("Target reached");
        //break;
    }
    
    delay(1000);
    //}

    delay(7000);
}

void turn_left_90()
{
    Serial.println("Turning left 90°.");

    // for (int i = 0; i < 5; i++)
    // {
    //     Serial.println(i);
    // }

    int newBearing = currentBearing + 90;
    if (newBearing >= 360) newBearing -= 360;
    
    Serial.print("Turning left 90° from bearing ");
    Serial.print(currentBearing);
    Serial.print("° to ");
    Serial.print(newBearing);
    Serial.println("°");
    
    analogWrite(FNA, 130 * LEFT_MOTOR_CALIBRATION);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 130 * RIGHT_MOTOR_CALIBRATION);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    float startYaw = yaw;
    float targetYaw = startYaw + 80.0;
    
    while (yaw < targetYaw) {
        updateMPU();
        Serial.print("Turning: Current Yaw = ");
        Serial.print(yaw);
        Serial.print(", Target = ");
        Serial.println(targetYaw);
        delay(10);
    }
    
    stopMotors();
    Serial.println("Rough turn complete");
    
    currentBearing = newBearing;
    alignToBearing(newBearing);

    delay(7000);
}

void turn_right_90()
{
    Serial.println("Turning right 90°.");

    // for (int i = 0; i < 5; i++)
    // {
    //     Serial.println(i);
    // }

    int newBearing = currentBearing - 90;
    if (newBearing < 0) newBearing += 360;
    
    Serial.print("Turning right 90° from bearing ");
    Serial.print(currentBearing);
    Serial.print("° to ");
    Serial.print(newBearing);
    Serial.println("°");
    
    analogWrite(FNA, 100 * LEFT_MOTOR_CALIBRATION);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, 100 * RIGHT_MOTOR_CALIBRATION);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    
    float startYaw = yaw;
    float targetYaw = startYaw - 80.0;
    
    while (yaw > targetYaw) {
        updateMPU();
        Serial.print("Turning: Current Yaw = ");
        Serial.print(yaw);
        Serial.print(", Target = ");
        Serial.println(targetYaw);
        delay(10);
    }
    
    stopMotors();
    Serial.println("Rough turn complete");
    delay(1000);
    
    currentBearing = newBearing;
    // Assume alignToBearing smoothly finalizes the turn
    // (see alignToBearing function below)
    // This call can help refine the turn if needed.
    alignToBearing(newBearing);

    delay(7000);
}

void turn_180()
{
    Serial.println("Turning 180°.");

    // for (int i = 0; i < 5; i++)
    // {
    //     Serial.println(i);
    // }

    int newBearing = currentBearing + 180;
    if (newBearing >= 360) newBearing -= 360;

    Serial.print("Turning 180° from bearing ");
    Serial.print(currentBearing);
    Serial.print("° to ");
    Serial.print(newBearing);
    Serial.println("°");

    analogWrite(FNA, 100 * LEFT_MOTOR_CALIBRATION);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 100 * RIGHT_MOTOR_CALIBRATION);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    float startYaw = yaw;
    float targetYaw = startYaw + 170.0;

    while (yaw < targetYaw) {
        updateMPU();
        delay(10);
    }

    stopMotors();
    Serial.println("Rough 180° turn complete");
    delay(1000);

    currentBearing = newBearing;
    alignToBearing(newBearing);

    delay(7000);
}

void maintainBearing() {
    Serial.println("Maintaining bearing");
    if (!correctionEnabled) return;
    
    float currentRelativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(currentRelativeYaw);
    
    float error = currentBearing - normalizedYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    const float BEARING_TOLERANCE = 3.0;
    
    if (abs(error) > BEARING_TOLERANCE) {
        Serial.print("Bearing correction needed. Error: ");
        Serial.println(error);
        
        int correctionPWM = min(abs(error) * 2, 50);
        
        if (isMovingForward) {
            if (error > 0) {
                analogWrite(FNA, (100 * LEFT_MOTOR_CALIBRATION) - correctionPWM);
                analogWrite(FNB, (100 * RIGHT_MOTOR_CALIBRATION) + correctionPWM);
            } else {
                analogWrite(FNA, (100 * LEFT_MOTOR_CALIBRATION) + correctionPWM);
                analogWrite(FNB, (100 * RIGHT_MOTOR_CALIBRATION) - correctionPWM);
            }
            delay(50);
            move_forward(100);
        } else {
            alignToBearing(currentBearing);
        }
    }
}

void updateDistance() {
    if (isMovingForward) {
        leftTotalDistance += (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        rightTotalDistance += (rightPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;
        Serial.print("Avg Distance: ");
        Serial.println(avgDistance);
        leftPulses = rightPulses = 0;
        if (avgDistance >= TARGET_DISTANCE) {
            stopMotors();
            targetReached = true;
        }
    }
}

void backtrack()
{
    Serial.println("Backtrack starting.");
    turn_180();
    delay(5000);


    for (count -= 1; count > junction_nodes[index]; count--)
    {
        Serial.print("Count: "); Serial.println(count);

        if (movement_arr[count] == 'F') 
        {
            Serial.println("Move forward now");
            move_forward(100);
        }
        else if (movement_arr[count] == 'L') 
        {
            Serial.println("Turn right now");
            turn_right_90();
        }
        else if (movement_arr[count] == 'R') 
        {
            Serial.println("Turn left now");
            turn_left_90();
        }
    }    

    if (movement_arr[junction_nodes[index]] == 'F')
    {
        Serial.println("Move forward now");
        move_forward(100);
    }
    count--;
}

void search_maze()
{
    Serial.println("Searching maze.");
    Serial.print("Currently in loop: "); Serial.println(count);

    int front, left, right;

    front = checkDist(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    left = checkDist(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    right = checkDist(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

    Serial.print("Front: "); Serial.println(front); 
    Serial.print("Left: "); Serial.println(left);
    Serial.print("Right: "); Serial.println(right);
    Serial.println("-----");
    Serial.println(movement_arr);
    Serial.println("-----");
    
    Serial.println("-----");
    Serial.print("Index: "); Serial.println(index);
    Serial.print("Junction visited: "); Serial.println(junction_visited[index]);
    Serial.println("-----");

    delay(5000);

    if (front == 1 && (count != junction_nodes[index] || junction_visited[index] < 1))                 // front has space
    {
        Serial.println("Front space detected");
        if (left == 1 || right == 1)
        {
            Serial.println("Left and/or right space detected");
            index++;
            junction_nodes[index] = count;
            Serial.println("Junction node stored.");
        }
       
        Serial.println("Move forward now.");
        move_forward(100);
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else if (left == 1 && (count != junction_nodes[index] || junction_visited[index] < 2))             // left has space
    {
        Serial.println("Left space detected");
        if (right == 1)
        {
            Serial.println("Right space detected");
            index++;
            junction_nodes[index] = count;
            Serial.println("Junction node stored.");
        }
        
        Serial.println("Turn left now");
        turn_left_90();
        Serial.println("Turn left 90° done.");
        movement_arr[count] = 'L';
        count++;
        Serial.println("Left turn stored.");
        
        Serial.println("Move forward now.");
        move_forward(100);
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else if (right == 1 && (count != junction_nodes[index] || junction_visited[index] < 3))           // right has space
    {
        Serial.println("Right space detected");
        
        Serial.println("Turn right now");
        turn_right_90();
        Serial.println("Turn right 90° done.");
        movement_arr[count] = 'R';
        count++;
        Serial.println("Right turn stored.");
        
        Serial.println("Move forward now.");
        move_forward(100);
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else
    {
        if ((junction_visited[index] == 2 && right == 0) || junction_visited[index] == 3)
        {
            Serial.println("All routes explored, removing junction.");
            junction_visited[index] = 0;
            index--;
        }

        backtrack();
        Serial.println("Backtracking complete.");
        count++;

        if (movement_arr[count] == 'F')
        {
            Serial.println("Turn 180 now");
            turn_180();
            Serial.println("Reorienting: F (Forward)");
            junction_visited[index] = 1;
            Serial.println("Junction visited stored as 1.");
        }
        else if (movement_arr[count] == 'L')
        {
            Serial.println("Turn left now");
            turn_left_90();
            Serial.println("Reorienting: L (Left)");
            junction_visited[index] = 2;
            Serial.println("Junction visited stored as 2.");
        }
        else if (movement_arr[count] == 'R')
        {
            Serial.println("Turn right now");
            turn_right_90();
            Serial.println("Reorienting: R (Right)");
            junction_visited[index] = 3;
            Serial.println("Junction visited stored as 3.");
        }
    }
}

void init_arrays()
{
    Serial.println("Initializing arrays.");

    memset(movement_arr, 0, sizeof(movement_arr));
    memset(junction_nodes, 0, sizeof(junction_nodes));
    memset(junction_visited, 0, sizeof(junction_visited));
}

void setup() 
{
    Serial.begin(9600);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(FNA, OUTPUT);
    pinMode(FNB, OUTPUT);
    ultrasonicSetup(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    ultrasonicSetup(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    ultrasonicSetup(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);
    Serial.println("Setup complete.");

    init_arrays();
}

void loop()
{
    search_maze();
}