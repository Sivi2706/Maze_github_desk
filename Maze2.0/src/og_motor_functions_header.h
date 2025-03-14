#ifndef OG_MOTOR_FUNCTIONS_H
#define OG_MOTOR_FUNCTIONS_H

// #pragma once

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

// Motor calibration to fix right-side drift
#define LEFT_MOTOR_CALIBRATION 1.0
#define RIGHT_MOTOR_CALIBRATION 0.95

// Encoder and distance calculation variables
extern volatile unsigned long leftPulses;
extern volatile unsigned long rightPulses;
extern const unsigned int PULSES_PER_TURN;
extern const float WHEEL_DIAMETER;
extern const float WHEEL_CIRCUMFERENCE;

// Distance tracking
extern float leftTotalDistance;
extern float rightTotalDistance;
extern bool isMovingForward;
extern bool targetReached;

// Bearing system variables
#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270
extern int currentBearing;
extern bool correctionEnabled;

// Correction timing variables
extern unsigned long lastCorrectionTime;
extern const unsigned long CORRECTION_INTERVAL;

// RPM calculation variables
extern unsigned long lastLeftPulseTime;
extern unsigned long lastRightPulseTime;
extern float leftRPM;
extern float rightRPM;

// MPU6050 Setup
extern const int MPU;
extern const float GYRO_SCALE;
extern float gyroX, gyroY, gyroZ;
extern float accelX, accelY, accelZ;
extern float accelAngleX, accelAngleY;
extern float roll, pitch, yaw;
extern float GyroErrorX, GyroErrorY, GyroErrorZ;
extern float elapsedTime, previousTime, currentTime;
extern float initialYaw;  // Store the initial yaw to use as reference
extern const float ALPHA; // Complementary filter coefficient

#define ROWS 8
#define COLS 8
#define SIZE (ROWS * COLS)

struct BooleanFlags {
    uint8_t isMovingForward : 1;
    uint8_t targetReached : 1;
    uint8_t correctionEnabled : 1;
    uint8_t is_LeBron_done : 1;
    uint8_t has_LeBron_written : 1;
    // uint8_t reduce_forward_dist_after_turn : 1;
};

extern BooleanFlags flags;

// Function declarations
void leftEncoderISR();
void rightEncoderISR();
void calculateRPM();
void ultrasonicSetup(int trigPin, int echoPin);
float getDistance(int trigPin, int echoPin);
int checkDist(int trigPin, int echoPin);
void getOrientation();
void getAcceleration();
void calculateError();
void updateMPU();
float normalizeYaw(float rawYaw);
int getCurrentAbsoluteBearing();
void turn_right_90();
void turn_left_90();
void turn_180();
void alignToBearing(int targetBearing);
void maintainBearing();
void MoveForward(int PWM);
void Forward25(int PWM);
void stopMotors();
void updateDistance();

#endif