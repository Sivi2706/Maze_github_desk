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
#define TARGET_DISTANCE 90.0

// Define number of rows and columns
#define ROWS 5
#define COLS 5

// Encoder and distance calculation variables
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;
const unsigned int PULSES_PER_TURN = 20;
const float WHEEL_DIAMETER = 4.0;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Distance tracking
float leftTotalDistance = 0.0;
float rightTotalDistance = 0.0;
bool isMovingForward = false;
bool targetReached = false;

// Interrupt service routines
void leftEncoderISR() { leftPulses++; }
void rightEncoderISR() { rightPulses++; }

// RPM calculation variables
unsigned long lastLeftPulseTime = 0;
unsigned long lastRightPulseTime = 0;
float leftRPM = 0.0;
float rightRPM = 0.0;

void calculateRPM() {
    unsigned long currentTime = millis();
    if (leftPulses > 0) {
        leftRPM = (leftPulses / (float)PULSES_PER_TURN) * (60000.0 / (currentTime - lastLeftPulseTime));
        lastLeftPulseTime = currentTime;
        leftPulses = 0;
    }
    if (rightPulses > 0) {
        rightRPM = (rightPulses / (float)PULSES_PER_TURN) * (60000.0 / (currentTime - lastRightPulseTime));
        lastRightPulseTime = currentTime;
        rightPulses = 0;
    }
}

// Ultrasonic Sensor Functions
void ultrasonicSetup(int trigPin, int echoPin) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long duration = pulseIn(echoPin, HIGH, 30000);
    return (duration == 0) ? 0 : (duration * 0.0343 / 2.0);
}

// MPU6050 Setup
const int MPU = 0x68;
const float GYRO_SCALE = 1.0 / 131.0;
float gyroX, gyroY, gyroZ;
float roll, pitch, yaw;
float GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, previousTime, currentTime;

void getOrientation() {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    gyroX = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    gyroY = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    gyroZ = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
}

void calculateError() {
    for (int i = 0; i < 200; i++) {
        getOrientation();
        GyroErrorX += gyroX;
        GyroErrorY += gyroY;
        GyroErrorZ += gyroZ;
    }
    GyroErrorX /= 200;
    GyroErrorY /= 200;
    GyroErrorZ /= 200;
    Serial.println("Gyroscope calibration complete.");
}

void updateMPU() {
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) * 0.001;
    getOrientation();
    gyroX -= GyroErrorX;
    gyroY -= GyroErrorY;
    gyroZ -= GyroErrorZ;
    roll += gyroX * elapsedTime;
    pitch += gyroY * elapsedTime;
    yaw += gyroZ * elapsedTime;
}

void move(int PWM, bool forward) {
    isMovingForward = forward;
    analogWrite(FNA, PWM);
    digitalWrite(IN1, forward);
    digitalWrite(IN2, !forward);
    analogWrite(FNB, PWM);
    digitalWrite(IN3, forward);
    digitalWrite(IN4, !forward);
    Serial.println(forward ? "Moving Forward..." : "Moving Backward...");
}

void stopMotors() {
    isMovingForward = false;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stopped.");
}

void updateDistance() {
    if (isMovingForward) {
        leftTotalDistance += (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        rightTotalDistance += (rightPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;
        Serial.print("Avg Distance: ");
        Serial.println(avgDistance);
        leftPulses = rightPulses = 0;
        if (avgDistance >= TARGET_DISTANCE && !targetReached) {
            stopMotors();
            targetReached = true;
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(FNA, OUTPUT);
    pinMode(FNB, OUTPUT);
    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
    attachPCINT(digitalPinToPCINT(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);
    ultrasonicSetup(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    ultrasonicSetup(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    ultrasonicSetup(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);
    Wire.begin();
    calculateError();
}

void loop() {
    //==========================ULTRASONIC SENSOR=========================================================
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

     //===========================MOTOR CONTROL + ROTARY ENCODER===========================================

    // // Move forward for 25 cm
    // // if (!targetReached) {
    // //     MoveForward(150);
    // // }

    // MoveForward(150);

    //    // Update and display distance traveled by the wheels
    // updateDistance();

    // // Calculate RPM
    // calculateRPM();

    // // Display data for Serial Plotter (comma-separated values)
    // Serial.print("LeftRaw:");
    // Serial.print(digitalRead(LEFT_ENCODER_PIN));
    // Serial.print(",RightRaw:");
    // Serial.print(digitalRead(RIGHT_ENCODER_PIN));
    // Serial.print(",LeftRPM:");
    // Serial.print(leftRPM);
    // Serial.print(",RightRPM:");
    // Serial.println(rightRPM); // Use println for the last value to end the line

    // // Display data for Serial Monitor (text-based)
    // Serial.print("Left Distance: ");
    // Serial.print(leftTotalDistance);
    // Serial.print(" cm | Right Distance: ");
    // Serial.print(rightTotalDistance);
    // Serial.print(" cm | Avg Distance: ");
    // Serial.print((leftTotalDistance + rightTotalDistance) / 2.0);
    // Serial.println(" cm");

    // // Stop when the target distance is reached
    // if ((leftTotalDistance + rightTotalDistance) / 2.0 >= 25.0 && !targetReached) {
    //     Stop();
    //     targetReached = true;
    // }



    //===============================MPU-6050==============================================================

    // Update MPU6050 angles
    updateMPU();

    // Print MPU6050 angles
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("° | Pitch: ");
    Serial.print(pitch);
    Serial.print("° | Yaw: "); //(Turning angle) negative for right turn and positive for left hand turn 
    Serial.print(yaw);
    Serial.println("°");

    delay(150);  // Slightly longer delay for smoother readings



}

// Initialise arrays to be used in maze movement
// **********************************************************************
// in next (2nd) draft, change these to local variables and use pointers 
// **********************************************************************
char temp_movement[ROWS * COLS];
char final_movement[ROWS * COLS];
int movement_index[ROWS * COLS];

// Initialise counters and flags
int index_counter = 0;
int prev_junction = 0;         // if last movement was forwards, set 1 ,    if last movement was left, set 2,   if right, set 3
int front, left, right;

int back_it_up_bih (int back_index)
{
    // function to make this bih do a 180º 
    // 8==================================D -----

    // **********************************************************************
    // test: trial and error weather i > OR i >= should be used 
    // **********************************************************************
    for (int i = back_index; i > movement_index[index_counter]; i--)
    {
        if (temp_movement[i] = 'F')
        {
            move(255, true);
        }
        else if (temp_movement[i] = 'L')
        {
            // function to TURN RIGHT *** OPPOSITE DIRECTION
        }
        else if (temp_movement[i] = 'R')
        {
            // function to TURN LEFT *** OPPOSITE DIRECTION
        }
    }

    // this bih has returned to most recent junction
    return movement_index[index_counter];
}

void keep_going_daddy ()
{
    for (int i = 0; i < ROWS * COLS; i++)
    {
        front = getDistance(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
        left = getDistance(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
        right = getDistance(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

        if (prev_junction >= 1 && front)          // if there is a baddie in front && the baddie has been visited before
        {
            if (left || right) movement_index[index_counter++] = i;     // save at which movement/node there is a junction
            move(255, true);
            temp_movement[i] = 'F'
        }
        else if (prev_junction >= 2 && left)      // if there is a baddie to the left && the baddie has been visited before
        {
            // function to turn left
            // 8==================================D -----
            move(255, true);
            temp_movement[i] = 'L'
        }
        else if (prev_junction >= 3 && right)     // if there is a baddie to the right && the baddie has been visited before
        {
            // function to turn right
            // 8==================================D -----
            move(255, true);
            temp_movement[i] = 'R'
        }
        else                // there is no baddie in front, left or right, i.e. a dead end
        {
            i = back_it_up_bih (i);

            // check which direction did this bih last take & 
            // set prev_junction flag to indicate which direction this bih should avoid
            if (temp_movement[i] = 'F')
            {
                // functon to make this bih to a 180º
                prev_junction = 1;
            }
            else if (temp_movement[i] = 'L')
            {
                // function to make this bih turn left
                prev_junction = 2;
            }
            else if (temp_movement[i] = 'R')
            {
                // function to nake this bih turn right
                prev_junction = 3;
            }
        }
    }
}