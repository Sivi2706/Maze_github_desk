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
#define TARGET_DISTANCE 25.0 //(in cm)

// Motor calibration to fix right-side drift
#define LEFT_MOTOR_CALIBRATION 1.0
#define RIGHT_MOTOR_CALIBRATION 1.0  // Reduce right motor speed if it's stronger

// Encoder and distance calculation variables
// extern const unsigned int PULSES_PER_TURN;
// extern const float WHEEL_DIAMETER;
// externconst float WHEEL_CIRCUMFERENCE;

// Bearing system variables
#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270
const float BEARING_TOLERANCE = 2.0;

// MPU6050 Setup
const int MPU = 0x68;
const float GYRO_SCALE = 1.0 / 131.0;
const float ALPHA = 0.96; // Complementary filter coefficient

const unsigned int PULSES_PER_TURN = 20;
const float WHEEL_DIAMETER = 4;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

#define ROWS 8
#define COLS 8
#define SIZE (ROWS * COLS)

extern char movement_arr[SIZE];
extern uint8_t junction_nodes[SIZE];
extern uint8_t junction_visited[SIZE];
extern uint8_t index;
extern uint8_t count;

struct BooleanFlags {
    uint8_t isMovingForward : 1;
    uint8_t targetReached : 1;
    uint8_t correctionEnabled : 1;
    uint8_t is_LeBron_done : 1;
    uint8_t has_LeBron_written : 1;
};

struct EncoderState {
    volatile unsigned long leftPulses = 0;
    volatile unsigned long rightPulses = 0;
    float leftTotalDistance = 0.0;
    float rightTotalDistance = 0.0;
};

struct MotorState {
    bool isMovingForward = false;
    bool targetReached = false;
};

struct MPUState {
    float gyroX, gyroY, gyroZ;
    float accelX, accelY, accelZ;
    float accelAngleX, accelAngleY;
    float roll, pitch, yaw;
    float GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
    float elapsedTime, previousTime, currentTime;
    float initialYaw = 0.0;  // Store the initial yaw to use as reference
};

struct BearingState {
    float currentRelativeBearing = 0.0;  // Start with 0 as default relative bearing
    bool correctionEnabled = true;
};

extern BooleanFlags flags;
extern EncoderState encoderState;
extern MPUState mpuState;
extern BearingState bearingState;
extern MotorState motorState;

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
void moveForwards(int PWM, MPUState &mpu, BearingState &bearing, MotorState &motor, EncoderState &encoder);
void turnLeft90(MPUState &mpu, BearingState &bearing);
void turnRight90(MPUState &mpu, BearingState &bearing);
void turn180(MPUState &mpu, BearingState &bearing);
void stopMotors();
void updateDistance(EncoderState &encoder, MotorState &motor);
float getDistance(int trigPin, int echoPin);
int checkDist(int trigPin, int echoPin);
void memoryReset();
void memoryWrite();
int memoryRead();
void backtrack_and_reorient();
void search_maze();
void init_arrays();