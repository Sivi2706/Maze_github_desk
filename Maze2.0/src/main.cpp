#include <Arduino.h>
#include <PinChangeInterrupt.h>

// Encoder pin definitions
#define LEFT_ENCODER_PIN  7
#define RIGHT_ENCODER_PIN 8

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
}

void MoveForward(int PWM) {
    if (!isMovingForward) {
        isMovingForward = true;
        analogWrite(FNA, PWM);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
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
    updateDistance();

    // Move forward if the target is not yet reached
    if (!targetReached) {
        MoveForward(150);
    }

    delay(150);  // Slightly longer delay for smoother readings
}
