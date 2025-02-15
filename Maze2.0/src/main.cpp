#include "NewPing.h"
#include <LiquidCrystal.h>

// Hook up HC-SR04 with Trig to Arduino Pin A4, Echo to Arduino Pin A5
#define TRIGGER_PIN A5
#define ECHO_PIN A4 

// Motor Pins
const int IN1 = 1;
const int IN2 = 10;
const int IN3 = 12;
const int IN4 = 13;
const int FNA = 3;
const int FNB = 11;

// LCD Pins
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Maximum distance we want to ping for (in centimeters)
#define MAX_DISTANCE 400	

// NewPing setup of pins and maximum distance
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

void setup() {
	Serial.begin(9600);

    // Motor Pin Setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(FNA, OUTPUT);
    pinMode(FNB, OUTPUT);

    // LCD initialization
    lcd.begin(16, 2);
    lcd.print("Initializing...");
    delay(1000);
    lcd.clear();
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
    Serial.print("Distance = ");
    Serial.print(distance);
    Serial.println(" cm");

    lcd.setCursor(0, 0);
    lcd.print("Distance: ");
    lcd.print(distance);
    lcd.print(" cm  ");

    if (distance > 10 || distance == 0) { // Move forward if no obstacle or out of range
        lcd.setCursor(0, 1);
        lcd.print("Moving forward");
        MoveForward(150); // Set motor speed to 150
    } else {
        lcd.setCursor(0, 1);
        lcd.print("Object detected!");
        Stop(); // Stop the car if object is within 10 cm

        // Use a while loop to ensure the car remains stopped
        while (distance <= 15 && distance != 0) {
            distance = sonar.ping_cm(); // Recheck the distance
            lcd.setCursor(0, 0);
            lcd.print("Distance: ");
            lcd.print(distance);
            lcd.print(" cm  ");

            lcd.setCursor(0, 1);
            lcd.print("Object detected!");
            delay(100); // Short delay to prevent rapid looping
        }
    }
    delay(100); // Short delay for smoother operation
}