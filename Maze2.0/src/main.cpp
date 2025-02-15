#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include "NewPing.h"

// Ultrasonic Sensor Pins
#define FRONT_TRIGGER_PIN A5
#define FRONT_ECHO_PIN A4
#define LEFT_TRIGGER_PIN A3
#define LEFT_ECHO_PIN A2
#define RIGHT_TRIGGER_PIN A1
#define RIGHT_ECHO_PIN A0

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

// Maximum distance for ultrasonic sensors (in centimeters)
#define MAX_DISTANCE 400

// NewPing objects for each ultrasonic sensor
NewPing front(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing left(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing right(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);

// Encoder and distance calculation variables
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;
const unsigned int PULSES_PER_TURN = 20;
const float WHEEL_DIAMETER = 6.5;  // in cm
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Total distance tracking
float totalDistance = 0;
bool isMovingForward = false;

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
    isMovingForward = true;
    analogWrite(FNA, PWM);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, PWM);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void Stop() {
    isMovingForward = false;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    // Reset total distance when stopping
    totalDistance = 0;
    Serial.println("Stopped - Distance reset to 0 cm");
}

void TurnLeft() {
    isMovingForward = false;
    analogWrite(FNA, 255);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 255);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    // Reset total distance when turning
    totalDistance = 0;
}

void TurnRight() {
    isMovingForward = false;
    analogWrite(FNA, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, 255);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    // Reset total distance when turning
    totalDistance = 0;
}

void updateDistance() {
    if (isMovingForward) {
        // Calculate distance for each wheel
        float leftDistance = (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        float rightDistance = (rightPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        
        // Update total distance with average of both wheels
        totalDistance += (leftDistance + rightDistance) / 2.0;
        
        // Print distance information
        Serial.print("Total Distance Traveled = ");
        Serial.print(totalDistance);
        Serial.println(" cm");
    }
    
    // Reset pulse counters
    leftPulses = 0;
    rightPulses = 0;
}

void loop() {
    // Read distance from front ultrasonic sensor
    int frontDistance = front.ping_cm();

    // Update and display distance only while moving forward
    updateDistance();

    // Display front sensor distance
    Serial.print("Front Distance = ");
    Serial.print(frontDistance);
    Serial.println(" cm");

    // Movement Logic
    if (frontDistance > 10 || frontDistance == 0) {
        // Keep moving forward
        MoveForward(150);
    } else {
        // Stop when obstacle detected at 10cm
        Stop();
        Serial.println("Obstacle detected! Stopping.");
    }

    delay(100); // Short delay for smoother operation
}