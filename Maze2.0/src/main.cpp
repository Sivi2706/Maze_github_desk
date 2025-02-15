#include "NewPing.h"

// Hook up HC-SR04 with Trig to Arduino Pin A4, Echo to Arduino Pin A5
#define TRIGGER_PIN A5
#define ECHO_PIN A4 

// Motor Pins
const int IN1 = 2;
const int IN2 = 4;
const int IN3 = 5;
const int IN4 = 6;
const int FNA = 3;
const int FNB = 11;

// Maximum distance we want to ping for (in centimeters)
#define MAX_DISTANCE 400	

// NewPing setup of pins and maximum distance
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
    Serial.begin(9600); // Initialize serial communication

    // Motor Pin Setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(FNA, OUTPUT);
    pinMode(FNB, OUTPUT);
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

void loop() {
    int distance = sonar.ping_cm(); // Get distance in cm

    // Display distance in serial monitor
    Serial.print("Distance = ");
    Serial.print(distance);
    Serial.println(" cm");

    if (distance > 10 || distance == 0) { // Move forward if no obstacle or out of range
        Serial.println("Moving forward");
        MoveForward(150); // Set motor speed to 150
    } else {
        Serial.println("Object detected!");
        Stop(); // Stop the car if object is within 10 cm

        // Use a while loop to ensure the car remains stopped
        while (distance <= 15 && distance != 0) {
            distance = sonar.ping_cm(); // Recheck the distance
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" cm");

            Serial.println("Object detected!");
            delay(100); // Short delay to prevent rapid looping
        }
    }
    delay(100); // Short delay for smoother operation
}