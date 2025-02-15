

#include "NewPing.h"

// Ultrasonic Sensor Pins
#define FRONT_TRIGGER_PIN A5
#define FRONT_ECHO_PIN A4
#define LEFT_TRIGGER_PIN A3  // Define pins for left sensor
#define LEFT_ECHO_PIN A2
#define RIGHT_TRIGGER_PIN A1 // Define pins for right sensor
#define RIGHT_ECHO_PIN A0
a
//Testing Durvish 

// Motor Pins
const int IN1 = 2;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 6;
const int FNA = 3;
const int FNB = 11;

// Maximum distance we want to ping for (in centimeters)
#define MAX_DISTANCE 400

// NewPing objects for each ultrasonic sensor
NewPing front(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing left(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing right(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);

// Rotary Encoder Variables
volatile int count_l = 0; // Counter for rotary encoder

// Function Prototype for Rotary Encoder ISR
void counter();

void setup() {
    Serial.begin(9600); // Initialize serial communication

    // Motor Pin Setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(FNA, OUTPUT);
    pinMode(FNB, OUTPUT);

    // Rotary Encoder Setup
    attachInterrupt(digitalPinToInterrupt(10), counter, RISING); // Attach interrupt for rotary encoder
}

void MoveForward(int PWM) {
    analogWrite(FNA, PWM);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, PWM);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void Stop() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void TurnLeft() {
    analogWrite(FNA, 255);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 255);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void TurnRight() {
    analogWrite(FNA, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, 255);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

// Rotary Encoder Interrupt Service Routine
void counter() {
    count_l++; // Increment counter on each rising edge
}

    // Read distances from ultrasonic sensors
    int frontDistance = front.ping_cm();
    int leftDistance = left.ping_cm();
    int rightDistance = right.ping_cm();

    // Display distances in serial monitor
    Serial.print("Front Distance = ");
    Serial.print(frontDistance);
    Serial.print(" cm | Left Distance = ");
    Serial.print(leftDistance);
    Serial.print(" cm | Right Distance = ");
    Serial.print(rightDistance);
    Serial.println(" cm");

    // Rotary Encoder Calculations
    double var_1 = count_l / 20.0; // Convert counts to rotations
    double var_2 = var_1 * 2 * 3.242 * 6.5; // Calculate distance traveled (example formula)
    Serial.print("Distance Traveled = ");
    Serial.print(var_2);
    Serial.println(" cm");

    // Obstacle Avoidance Logic
    if (frontDistance > 10 || frontDistance == 0) { // Move forward if no obstacle in front
        Serial.println("Moving forward");
        MoveForward(150); // Set motor speed to 150
    } else if (leftDistance > rightDistance) { // Turn left if more space on the left
        Serial.println("Turning left");
        TurnLeft();
        delay(500); // Turn for a short duration
        Stop();
    } else { // Turn right if more space on the right
        Serial.println("Turning right");
        TurnRight();
        delay(500); // Turn for a short duration
        Stop();
    }

    delay(100); // Short delay for smoother operation
