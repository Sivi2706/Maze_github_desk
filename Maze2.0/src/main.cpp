#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>

// Ultrasonic Sensor Pins
#define FRONT_TRIGGER_PIN 10  // Changed to pin 10
#define FRONT_ECHO_PIN 9      // Changed to pin 9
#define LEFT_TRIGGER_PIN A3
#define LEFT_ECHO_PIN A2
#define RIGHT_TRIGGER_PIN A0
#define RIGHT_ECHO_PIN A1

// Encoder pin definitions
#define LEFT_ENCODER_PIN 13  // Changed from on-board LED to rotary encoder
#define RIGHT_ENCODER_PIN 12

// Motor Pins
#define IN1 2
#define IN2 4
#define IN3 5
#define IN4 6
#define FNA 3
#define FNB 11

// Target distance to travel (in centimeters)
#define TARGET_DISTANCE 90.0

// Encoder and distance calculation variables
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;
const unsigned int PULSES_PER_TURN = 20;
const float WHEEL_DIAMETER = 4;  // in cm
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Separate distance tracking for each wheel
float leftTotalDistance = 0.0;
float rightTotalDistance = 0.0;
bool isMovingForward = false;
bool targetReached = false;

// Interrupt service routines for encoders
void leftEncoderISR() {
    leftPulses++;
}

void rightEncoderISR() {
    rightPulses++;
}

// Ultrasonic Sensor Functions
void ultrasonicSetup(int trigPin, int echoPin) {
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);
}

// MPU6050 I2C address
const int MPU = 0x68;

// Gyro scale factor
const float GYRO_SCALE = 1.0 / 131.0;

// Variables to hold gyro outputs
float gyroX, gyroY, gyroZ;

// Variables to hold errors for calibration
float GyroErrorX, GyroErrorY, GyroErrorZ;

// Variables to hold angles
float roll, pitch, yaw;

// Timing variables
float elapsedTime, previousTime, currentTime;

// Function to read raw gyro data from MPU6050
void getOrientation() {
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Start reading from register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 bytes (2 bytes per axis)

    // Read raw gyro data for X, Y, and Z axes
    gyroX = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE; // X-axis
    gyroY = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE; // Y-axis
    gyroZ = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE; // Z-axis
}

// Function to calculate gyro errors for calibration
void calculateError() {
    byte c = 0;
    GyroErrorX = 0;
    GyroErrorY = 0;
    GyroErrorZ = 0;

    // Read gyro values 200 times and accumulate errors
    while (c < 200) {
        getOrientation();
        GyroErrorX += gyroX;
        GyroErrorY += gyroY;
        GyroErrorZ += gyroZ;
        c++;
    }

    // Calculate average error for each axis
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;

    Serial.println("Gyroscope calibration complete.");
}

// MPU6050 setup function
void mpuSetup() {
    Wire.begin();                      // Initialize I2C communication
    Wire.beginTransmission(MPU);       // Start communication with MPU6050
    Wire.write(0x6B);                  // Talk to the PWR_MGMT_1 register (6B)
    Wire.write(0x00);                  // Reset the MPU6050
    Wire.endTransmission(true);        // End the transmission

    // Calibrate the gyroscope
    calculateError();
}

// Function to update roll, pitch, and yaw angles
void updateMPU() {
    // Calculate elapsed time
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) * 0.001; // Convert to seconds

    // Read gyro data
    getOrientation();

    // Correct gyro outputs with calculated error values
    gyroX -= GyroErrorX;
    gyroY -= GyroErrorY;
    gyroZ -= GyroErrorZ;

    // Calculate angles for roll, pitch, and yaw
    roll += gyroX * elapsedTime;
    pitch += gyroY * elapsedTime;
    yaw += gyroZ * elapsedTime;

    // Round angles to 1 decimal place
    roll = round(roll * 10) / 10.0;
    pitch = round(pitch * 10) / 10.0;
    yaw = round(yaw * 10) / 10.0;
}

void setup() {
    Serial.begin(9600);

    // Motor Pin Setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(FNA, OUTPUT);
    pinMode(FNB, OUTPUT);

    // Configure encoder pins
    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

    // Initialize interrupts for encoders
    attachPCINT(digitalPinToPCINT(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
    attachPCINT(digitalPinToPCINT(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

    // Initialize ultrasonic sensors
    ultrasonicSetup(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    ultrasonicSetup(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    ultrasonicSetup(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

    // Initialize MPU6050
    mpuSetup();
}

void MoveForward(int PWM) {
    if (!isMovingForward) {
        isMovingForward = true;
        analogWrite(FNA, PWM);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(FNB, PWM);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        Serial.println("Moving Forward...");
    }
}

void Stop() {
    isMovingForward = false;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stopped - Left Distance: " + String(leftTotalDistance) + " cm | Right Distance: " + String(rightTotalDistance) + " cm");
}

void TurnLeft() {
    isMovingForward = false;
    analogWrite(FNA, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, 255);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void TurnRight() {
    isMovingForward = false;
    analogWrite(FNA, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, 255);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void updateDistance() {
    if (isMovingForward) {
        // Calculate distance for each wheel
        float leftDistance = (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        float rightDistance = (rightPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        
        // Update total distance for each wheel
        leftTotalDistance += leftDistance;
        rightTotalDistance += rightDistance;

        // Compute the average distance
        float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;

        // Print distance information
        Serial.print("Left: ");
        Serial.print(leftTotalDistance);
        Serial.print(" cm | Right: ");
        Serial.print(rightTotalDistance);
        Serial.print(" cm | Avg: ");
        Serial.print(avgDistance);
        Serial.println(" cm");

        // Reset pulse counters
        leftPulses = 0;
        rightPulses = 0;

        // Stop when the average distance reaches the target
        if (avgDistance >= TARGET_DISTANCE && !targetReached) {
            Stop();
            targetReached = true;
        }
    }
}

// Return distance in cm
float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH);
    if (duration == 0) {
        Serial.println("Error: No echo received. Check sensor connections or object distance.");
        return 0;  // Return 0 if no echo is received
    }
    return (duration * 0.034613 / 2.00);  // Convert pulse duration to distance in cm
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

    // MoveForward(150);

    //// Update and display distance traveled by the wheels
    // updateDistance();

    //===============================MPU-6050==============================================================
    // Update MPU6050 angles
    updateMPU();

    // Print MPU6050 angles
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("° | Pitch: ");
    Serial.print(pitch);
    Serial.print("° | Yaw: ");
    Serial.print(yaw);
    Serial.println("°");

    delay(150);  // Slightly longer delay for smoother readings
}