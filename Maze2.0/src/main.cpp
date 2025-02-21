#include <Arduino.h>
#include <PinChangeInterrupt.h>

// Ultrasonic Sensor Pins
#define FRONT_TRIGGER_PIN A4
#define FRONT_ECHO_PIN A5
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

void loop() {
    //==========================ULTRASONIC SENSOR==================================================================

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
    
    //===========================MOTOR CONTROL + ROTARY ENCODER====================================================================

    // // Move forward indefinitely
    // TurnLeft();

    // // Update and display distance traveled by the wheels
    // updateDistance();

    // delay(150);  // Slightly longer delay for smoother readings

    //===================================================================================================

}