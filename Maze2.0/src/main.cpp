#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>

// Ultrasonic Sensor Pins
#define FRONT_TRIGGER_PIN 10
#define FRONT_ECHO_PIN 9
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
#define TARGET_DISTANCE 25.0 //(in cm)

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

// Interrupt service routines
void leftEncoderISR() { leftPulses++; }
void rightEncoderISR() { rightPulses++; }

// RPM calculation variables
unsigned long lastLeftPulseTime = 0;
unsigned long lastRightPulseTime = 0;
float leftRPM = 0.0;
float rightRPM = 0.0;

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

// Ultrasonic Sensor Functions
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

// MPU6050 Setup
const int MPU = 0x68;
const float GYRO_SCALE = 1.0 / 131.0;
float gyroX, gyroY, gyroZ;
float roll, pitch, yaw;
float GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, previousTime, currentTime;

void getOrientation() {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    gyroX = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    gyroY = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    gyroZ = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
}

void calculateError() {
    for (int i = 0; i < 200; i++) {
        getOrientation();
        GyroErrorX += gyroX;
        GyroErrorY += gyroY;
        GyroErrorZ += gyroZ;
    }
    GyroErrorX /= 200;
    GyroErrorY /= 200;
    GyroErrorZ /= 200;
    Serial.println("Gyroscope calibration complete.");
}

void updateMPU() {
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) * 0.001;
    getOrientation();
    gyroX -= GyroErrorX;
    gyroY -= GyroErrorY;
    gyroZ -= GyroErrorZ;
    roll += gyroX * elapsedTime;
    pitch += gyroY * elapsedTime;
    yaw += gyroZ * elapsedTime;
}

// Turn functions using MPU6050 yaw readings
void turnRight90() {
    float initialYaw = yaw;
    float targetYaw = initialYaw - 90.0;  // For right turn, decrease yaw by 90 degrees
    
    Serial.println("Turning right 90 degrees...");
    Serial.print("Initial Yaw: ");
    Serial.print(initialYaw);
    Serial.print(" | Target Yaw: ");
    Serial.println(targetYaw);
    
    // Start turning right
    analogWrite(FNA, 100);  // Left motor forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, 100);  // Right motor backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    
    // Continue turning until we reach the target yaw
    while (yaw > targetYaw) {
        updateMPU();  // Update MPU readings
        
        // Print current yaw for debugging
        Serial.print("Current Yaw: ");
        Serial.print(yaw);
        Serial.print(" | Target: ");
        Serial.println(targetYaw);
        
        delay(10);  // Small delay for stability
    }
    
    // Stop motors once we've reached the target
    stopMotors();
    Serial.println("Right turn complete.");
}

void turnLeft90() {
    float initialYaw = yaw;
    float targetYaw = initialYaw + 90.0;  // For left turn, increase yaw by 90 degrees
    
    Serial.println("Turning left 90 degrees...");
    Serial.print("Initial Yaw: ");
    Serial.print(initialYaw);
    Serial.print(" | Target Yaw: ");
    Serial.println(targetYaw);
    
    // Start turning left
    analogWrite(FNA, 100);  // Left motor backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 100);  // Right motor forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    // Continue turning until we reach the target yaw
    while (yaw < targetYaw) {
        updateMPU();  // Update MPU readings
        
        // Print current yaw for debugging
        Serial.print("Current Yaw: ");
        Serial.print(yaw);
        Serial.print(" | Target: ");
        Serial.println(targetYaw);
        
        delay(10);  // Small delay for stability
    }
    
    // Stop motors once we've reached the target
    stopMotors();
    Serial.println("Left turn complete.");
}


void MoveForward(int PWM) {
    isMovingForward = true;  // Fixed: Was using undefined variable 'forward'
    analogWrite(FNA, PWM);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, PWM);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving Forward...");  // Added feedback message
}


void stopMotors() {
    isMovingForward = false;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stopped.");
}

void updateDistance() {
    if (isMovingForward) {
        leftTotalDistance += (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        rightTotalDistance += (rightPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;
        Serial.print("Avg Distance: ");
        Serial.println(avgDistance);
        leftPulses = rightPulses = 0;
        if (avgDistance >= TARGET_DISTANCE && !targetReached) {
            stopMotors();
            targetReached = true;
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(FNA, OUTPUT);
    pinMode(FNB, OUTPUT);
    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
    attachPCINT(digitalPinToPCINT(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);
    ultrasonicSetup(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    ultrasonicSetup(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    ultrasonicSetup(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);
    Wire.begin();
    Wire.beginTransmission(MPU);  // Added: Initialize MPU communication
    Wire.write(0x6B);             // PWR_MGMT_1 register
    Wire.write(0);                // Wake up the MPU-6050
    Wire.endTransmission(true);
    calculateError();
    
    // Reset distance counters
    leftTotalDistance = 0.0;
    rightTotalDistance = 0.0;
    targetReached = false;
}

void loop() {
//==========================ULTRASONIC SENSOR=========================================================
    // // Read distances from all three ultrasonic sensors
    // float frontDistance = getDistance(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    // float leftDistance = getDistance(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    // float rightDistance = getDistance(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

    // // Display ultrasonic sensor readings
    // Serial.print("Front: ");
    // Serial.print(frontDistance);
    // Serial.print(" cm | Left: ");
    // Serial.print(leftDistance);
    // Serial.print(" cm | Right: ");
    // Serial.print(rightDistance);
    // Serial.println(" cm");

//===========================MOTOR CONTROL + ROTARY ENCODER===========================================

    // //Move forward for 25 cm
    // if (!targetReached) {  // Added check to prevent continuous movement after target is reached
    //     MoveForward(255);
    // }

    // // Update and display distance traveled by the wheels
    // updateDistance();

    // // Calculate RPM
    // calculateRPM();

    // // Display data for Serial Plotter (comma-separated values)
    // Serial.print("LeftRaw:");
    // Serial.print(digitalRead(LEFT_ENCODER_PIN));
    // Serial.print(",RightRaw:");
    // Serial.print(digitalRead(RIGHT_ENCODER_PIN));
    // Serial.print(",LeftRPM:");
    // Serial.print(leftRPM);
    // Serial.print(",RightRPM:");
    // Serial.println(rightRPM); // Use println for the last value to end the line

    // // Display data for Serial Monitor (text-based)
    // Serial.print("Left Distance: ");
    // Serial.print(leftTotalDistance);
    // Serial.print(" cm | Right Distance: ");
    // Serial.print(rightTotalDistance);
    // Serial.print(" cm | Avg Distance: ");
    // Serial.print((leftTotalDistance + rightTotalDistance) / 2.0);
    // Serial.println(" cm");

    // // Stop when the target distance is reached
    // if ((leftTotalDistance + rightTotalDistance) / 2.0 >= 25.0 && !targetReached) {
    //     stopMotors();
    //     targetReached = true;
    // }

//===============================25cm===================================================================

    // if (!isMovingForward && !targetReached) {
    //     // Reset distance counters before movement starts
    //     leftTotalDistance = 0.0;
    //     rightTotalDistance = 0.0;
    //     targetReached = false;

    //     // Start moving
    //     MoveForward(100);
    // }

    // // Update distance traveled
    // updateDistance();

    // // Check if the car has reached exactly 25 cm
    // float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;
    // if (avgDistance >= 25.0 && !targetReached) {
    //     stopMotors();
    //     targetReached = true;
    // }

    // // Print distance values for debugging
    // Serial.print("Left Distance: ");
    // Serial.print(leftTotalDistance);
    // Serial.print(" cm | Right Distance: ");
    // Serial.print(rightTotalDistance);
    // Serial.print(" cm | Avg Distance: ");
    // Serial.print(avgDistance);
    // Serial.println(" cm");

    // delay(10);  // Small delay to ensure smooth execution

//===============================MPU-6050==============================================================

    //     // Update MPU6050 angles
    //     updateMPU();

    //     // Print MPU6050 angles
    //     Serial.print("Roll: ");
    //     Serial.print(roll);
    //     Serial.print("° | Pitch: ");
    //     Serial.print(pitch);
    //     Serial.print("° | Yaw: "); //(Turning angle) negative for right turn and positive for left hand turn 
    //     Serial.print(yaw);
    //     Serial.println("°");

    //    delay(100);  // Added delay for stability

//=================================90 RIGHT & LEFT============================================================
    // // Update MPU6050 angles
    // updateMPU();
    
    // // Only print yaw for turning reference
    // Serial.print("Yaw: ");
    // Serial.print(yaw);
    // Serial.println("°");
    
    // // Static variable to ensure the turn only happens once
    // static bool turnCompleted = false;
    
    // // Turn left 90 degrees once, then stop
    // if (!turnCompleted) {
    //     turnRight90();  // This function already stops the motors after turning
    //     turnLeft90();
    //     turnCompleted = true;
    //     Serial.println("Turn and stop sequence completed");
    // }
    
    // // Keep updating distance if moving forward
    // if (isMovingForward) {
    //     updateDistance();
    // }
    
    // delay(50);  // Smaller delay for more responsive command detection

}