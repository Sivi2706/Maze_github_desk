#ifndef MOTOR_FUNCTIONS_H
#define MOTOR_FUNCTIONS_H

#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>

// Ultrasonic Sensor Pins
#define FRONT_TRIGGER_PIN 7
#define FRONT_ECHO_PIN 8
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
#define TARGET_DISTANCE 23.0
#define TARGET_DISTANCE2 12.0

// Motor calibration to fix right-side drift
#define LEFT_MOTOR_CALIBRATION 1.5
#define RIGHT_MOTOR_CALIBRATION 1.0

// MPU6050 Constants
#define MPU 0x68
#define GYRO_SCALE 0.00763
#define ALPHA 0.98
#define BEARING_TOLERANCE 5.0

// Encoder and distance calculation variables
const unsigned int PULSES_PER_TURN = 20;
const float WHEEL_DIAMETER = 4;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

struct MPUState {
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
    float GyroErrorX, GyroErrorY, GyroErrorZ;
    float accelAngleX, accelAngleY;
    float roll, pitch, yaw;
    float initialYaw;
    unsigned long currentTime, previousTime;
    float elapsedTime;
};

struct BearingState {
    float currentRelativeBearing;
    bool correctionEnabled;
};

struct MotorState {
    bool isMovingForward;
    bool targetReached;
};

struct EncoderState {
    volatile int leftPulses, rightPulses;
    float leftTotalDistance, rightTotalDistance;
};

#define ROWS 8
#define COLS 8
#define SIZE (ROWS * COLS)

extern char movement_arr[SIZE];
extern uint8_t junction_nodes[SIZE];
extern uint8_t junction_visited[SIZE];
extern uint8_t index;
extern uint8_t count;

struct BooleanFlags {
    unsigned int isMovingForward : 1;
    unsigned int targetReached : 1;
    unsigned int correctionEnabled : 1;
    unsigned int is_LeBron_done : 1;
    unsigned int has_LeBron_written : 1;
};

extern BooleanFlags flags;
extern EncoderState encoderState;
extern MotorState motorState;
extern MPUState mpuState;
extern BearingState bearingState;

// Function Declarations
void leftEncoderISR();
void rightEncoderISR();
void ultrasonicSetup(int trigPin, int echoPin);
void calculateError(MPUState &mpu);
void getOrientation(MPUState &mpu);
void getAcceleration(MPUState &mpu);
void updateMPU(MPUState &mpu);
float normalizeYaw(float rawYaw);
float getCurrentRelativeBearing(const MPUState &mpu, const BearingState &bearing);
void printCurrentBearing(const MPUState &mpu, const BearingState &bearing);
void alignToBearing(MPUState &mpu, BearingState &bearing, float targetRelativeBearing);
void maintainBearing(MPUState &mpu, BearingState &bearing, MotorState &motor);
void MoveForward(int PWM);
void Forward25(MPUState &mpu, BearingState &bearing, MotorState &motor, EncoderState &encoder, int trigPin1, int echoPin1, int trigPin2, int echoPin2, int trigPin3, int echoPin3);
//void moveForwards(int PWM, MPUState &mpu, BearingState &bearing, MotorState &motor, EncoderState &encoder);
void ultraForwards(int PWM, MPUState &mpu, BearingState &bearing, MotorState &motor, EncoderState &encoder);
//void Forward25(MPUState &mpu, BearingState &bearing, MotorState &motor, EncoderState &encoder);
void turn_left_90(MPUState &mpu, BearingState &bearing);
void turn_right_90(MPUState &mpu, BearingState &bearing);
void turn_180(MPUState &mpu, BearingState &bearing);
void stopMotors();
void updateDistance(EncoderState &encoder, MotorState &motor);
float getDistance(int trigPin, int echoPin);
int checkDist(int trigPin, int echoPin);

#endif // MOTOR_FUNCTIONS_H
