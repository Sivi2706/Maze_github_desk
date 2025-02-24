#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include "NewPing.h"

/*
Improvements: 
1. Implementing MPU to turn the car rather than using delay
*/

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
const float WHEEL_DIAMETER = 4;  // in cm
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Separate distance tracking for each wheel
float leftTotalDistance = 0.0;
float rightTotalDistance = 0.0;
bool isMovingForward = false; //optional?
int direction = 2; // 1 = left, 2 = front, 3 = right, 4 = 180 reverse
bool targetReached = false;

void leftEncoderISR();
void rightEncoderISR();
void forward(int PWM);
void stop();
void left();
void right();
void reverse();

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

// Interrupt service routines for encoders
void leftEncoderISR() {
    leftPulses++;
}

void rightEncoderISR() {
    rightPulses++;
}

void forward(int PWM) {
    analogWrite(FNA, PWM);
    analogWrite(FNB, PWM);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving Forwards");
}

void stop() {
    analogWrite(FNA, 0);
    analogWrite(FNB, 0);
    Serial.println("Stop");
}

void right() {
    analogWrite(FNA, 255);
    analogWrite(FNB, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving right");
}

void left() {
    analogWrite(FNA, 255);
    analogWrite(FNB, 255);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Moving left");
}

void reverse() { //instead of reversing, maybe should turn 180 and just move forwards 
    analogWrite(FNA, 255);
    analogWrite(FNB, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Reversing");
}

void updateDistance() {
    float leftDistance = (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
    float rightDistance = (rightPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;

    //check the direction that the car is moving and then decide if add or minus distance
    //use the distance to determine if the car has moved up a 'block'
    //can also use ultrasonic and distance together to make it more accurate

    float avgDistance = (leftTotalDistance + rightTotalDistance) / 2;
    
    if (direction == 4) {
        leftTotalDistance -= avgDistance;
        rightTotalDistance -= avgDistance;
    } else {
        leftTotalDistance += avgDistance;
        rightTotalDistance += avgDistance;
    }

    leftPulses = 0;
    rightPulses = 0;
}

//why so hard ah this system
/*oweirjwoeirjowiejroiwejr
woeirjwoierjwoierjwoierjowierjoiwj
woeirjweoirjwoeirjwoierjwoierjowierj
woierjwoierjwoiejrwoeirjwoeirjwoierj
woeirjwoeirjweoirjwoeirjweoirjwoierjwoeirjwoierjwoierj
woeirjwoeirjwoeirjwoeirjwoierj
weroijweorijweorijweorijweorijweorijweorijwoeirj
weorijweorijweorijweorijweorijwoeirj*/