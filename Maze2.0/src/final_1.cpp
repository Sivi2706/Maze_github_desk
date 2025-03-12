// finalized version of the code (hopefully) with optimized algorithm and functioning motor movements

#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <EEPROM.h>

#define FRONT_TRIGGER_PIN 7
#define FRONT_ECHO_PIN 8
#define LEFT_TRIGGER_PIN A3
#define LEFT_ECHO_PIN A2
#define RIGHT_TRIGGER_PIN A0
#define RIGHT_ECHO_PIN A1

#define LEFT_ENCODER_PIN 13
#define RIGHT_ENCODER_PIN 12

#define IN1 2 
#define IN2 4
#define IN3 5 
#define IN4 6
#define FNA 3
#define FNB 11

#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270

#define ROWS 8
#define COLS 8
#define MAX_ELEMENTS ROWS * COLS

//Target distance to travel
#define TARGET_DISTANCE 25

//Calibrates the drifting of both motors (could be removed since we already use MPU for aligning)
#define LEFT_MOTOR_CALIBRATION 1.0
#define RIGHT_MOTOR_CALIBRATION 0.95

//Variables for encoders
unsigned int leftPulses = 0; //switched from long to int, 4b to 2b, saves 4b in total
unsigned int rightPulses = 0;
int PULSES_PER_TURN = 20;
float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

//Distance tracking
float leftTotalDistance = 0.0;
float rightTotalDistance = 0.0;

//Bearing system variables
int currentBearing = NORTH;

//Correction (aligning) timing variables
long lastCorrectionTime = 0;
long CORRECTION_INTERVAL = 500;

//MPU6050 setup
int MPU = 0x68;
//float GYRO_SCALE = 1.0 / 131.0; //could be added into code instead to reduce SRAM usage
float gyroX, gyroY, gyroZ; 
float accelX, accelY, accelZ;
//float accelAngleX, accelAngleY; //could be added into code instead to reduce SRAM usage
float roll, pitch, yaw;
float GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, previousTime, currentTime; 
float initialYaw = 0.0;
//float ALPHA = 0.96; //could be added into code instead to reduce SRAM usage

//Flags to store bools
struct BooleanFlags {
    unsigned int isMovingForward = 0;
    unsigned int targetReached = 0;
    unsigned int correctionEnabled = 1;
}

//ISP for rotary encoders
void leftEncoderISP() {leftPulses++;}
void rightEncoderISP() {rightPulses++;}

//setup ultrasonic sensors in initial setup() function
void ultrasonicSetup(int trigPin, int echoPin) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, OUTPUT);
}

//returns distance using ultrasonic sensors
float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn 0 : (duration * 0.0343 / 2.0);
}

//returns true or false depending on the distance from ultrasonic sensors
int checkDist(int trigPin, int echoPin) {
    if (getDistance(trigPin, echoPin) > 210) return -1; 
    return getDistance(trigPin, echoPin) <= 5 ? 0 : 1;
}

//updates the orientation of the MPU
void getOrientation() {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    gyroX = (Wire.read() << 8 | Wire.read()) * 1.0 / 131.0;
    gyroY = (Wire.read() << 8 | Wire.read()) * 1.0 / 131.0;
    gyroZ = (Wire.read() << 8 | Wire.read()) * 1.0 / 131.0;
}

//updates the acceleration values of the MPU
void getAcceleration() {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    accelX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

//calculates the error in the gyroscope
void calculateError() {
    for (int i = 0; i < 1000; i++) {
        getOrientation();
        GyroErrorX += gyroX;
        GyroErrorY += gyroY;
        GyroErrorZ += gyroZ;
        delay(1);
    }
    GyroErrorX += gyroX;
    GyroErrorY += gyroY;
    GyroErrorZ += gyroZ;
    Serial.println(F("Gyroscope calibration complete."));
}

//returns the current yaw of the MPU
void updateMPU() {
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) * 0.001;

    getOrientation();
    getAcceleration();

    gyroX -= GyroErrorX;
    gyroY -= GyroErrorY;
    gyroZ -= GyroErrorZ;
 
    float accelAngleX = atan2(accelY, sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI;
    float accelAngleY = atan2(-accelX, sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI;

    roll = 0.96 * (roll + gyroX * elapsedTime) + (1 - 0.96) * accelAngleX;
    pitch = 0.96 * (pitch + gyroY * elapsedTime) + (1 - 0.96) * accelAngleY;

    yaw += gyroZ * elapsedTime
}

//converts the angle to within 0 - 360
float normalizeYaw(float rawYaw) {
    float normalized = fmod(rawYaw, 360.0);
    if (normalized < 0) normalized += 360.0;
    return normalized;
}

//returns the current bearing of the car
int getCurrentAbsoluteBearing() {
    float relativeYaw = yaw - initialYaw;

    float normalizedYaw = normalizeYaw(relativeYaw);

    int absoluteBearing = round(normalizedYaw / 90) * 90;
    if (absoluteBearing >= 360) absoluteBearing = 0;

    return absoluteBearing;
}

//aligns the car to a certain bearing (not as accurate...?)
void alignToBearing(int targetBearing) {
    int currentAbsoluteBearing = getCurrentAbsoluteBearing();
    float currentRelativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(currentRelativeYaw);

    float error = targetBearing - normalizedYaw;

    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    Serial.print(F("Aligning to bearing: "));
    Serial.print(targetBearing);

    float BEARING_TOLERANCE = 5.0;

    if (abs(error) > BEARING_TOLERANCE) {
        if (error > 0) {
            analogWrite(FNA, 75);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(FNB, 75);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else {
            analogWrite(FNA, 75);
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(FNB, 75);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        }

        float currentError;
        do {
            updateMPU();

            currentRelativeYaw = yaw - initialYaw;
            normalizedYaw = normalizeYaw(currentRelativeYaw);
            currentError = targetBearing - normalizedYaw;

            if (currentError > 180) currentError -= 360;
            if (currentError < -180) currentError += 360;

            delay(5);
        } while (abs(currentError) > BEARING_TOLERANCE * 0.5);

        stopMotors();

        currentBearing = targetBearing;
    }
}

//aligns the car to a specific bearing for a set time, is more accurate
void maintainBearing() {
    if (BooleanFlags.correctionEnabled == 0) return;

    float currentRelativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(currentRelativeYaw);

    float error = currentBearing - normalizedYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    const float BEARING_TOLERANCE = 3.0;

    if (abs(error) > BEARING_TOLERANCE) {
        int correctionPWM = min(abs(error) * 2, 50);

        if (BooleanFlags.isMovingForward == 1) {
            if (error > 0) {
                analogWrite(FNA, 100 - correctionPWM);
                analogWrite(FNB, 100 - correctionPWM);
            } else {
                analogWrite(FNA, 100 + correctionPWM);
                analogWrite(FNB, 100 - correctionPWM);
            }

            delay(50);

            MoveForward(100);
        } else {
            alignToBearing(currentBearing);
        }
    }
}

void updateDistance() {
    if (BooleanFlags.isMovingForward) {
        leftTotalDistance += (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        rightTotalDistance += (rightPulses / (float)PULSES_PER_TURN) *WHEEL_CIRCUMFERENCE;
        float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;
        Serial.print(F("Avg distance: "));
        Serial.println(avgDistance);
        leftPulses = rightPulses = 0;
        if (avgDistance >= TARGET_DISTANCE && BooleanFlags.targetReached == 0) {
            stopMotors();
            BooleanFlags.targetReached = 1;
        }
    }
}



// void turnRight90() { //old movement code, does not align to bearing accurately
//     int newBearing = currentBearing - 90;
//     if (newBearing < 0) newBearing += 360;

//     Serial.print(F("Turning right 90 to"));
//     Serial.println(newBearing);

//     analogWrite(FNA, 100);
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);
//     analogWrite(FNB, 100);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, HIGH);

//     float startYaw = yaw;
//     float targetYaw = startYaw - 70.0;

//     while (yaw > targetYaw) {
//         updateMPU();

//         delay(5);
//     }

//     stopMotors();
//     Serial.println(F("Rough turn complete"));

//     currentBearing = newBearing;

//     for (int i = 0; i < 100; i++) {
//         Serial.println(F("Aligning"));
//         alignToBearing(newBearing);
//         delay(5);
//     }
// }

// void turnLeft90() { //old turn left code, does not align to bearing accurately
//     int newBearing = currentBearing + 90;
//     if (newBearing >= 360) newBearing -= 360;

//     Serial.print(F("Turning left 90 to "));
//     Serial.println(newBearing);

//     analogWrite(FNA, 100);
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, HIGH);
//     analogWrite(FNB, 100);
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);

//     float startYaw = yaw;
//     float targetYaw = startYaw + 70.0;

//     while (yaw < targetYaw) {
//         updateMPU();

//         delay(5);
//     }

//     stopMotors();
//     Serial.println(F("Rough turn complete"));

//     currentBearing = newBearing;

//     for (int i = 0; i < 6; i++) {
//         alignToBearing(newBearing);
//     }
// }

// void turn180() { //old code for turning 180, does not align to bearing accurately
//     int newBearing = currentBearing + 180;
//     if (newBearing >= 360) newBearing -= 360;

//     Serial.print(F("Turning 180 from bearing "));
//     Serial.println(newBearing);

//     analogWrite(FNA, 100);
//     digitalwrite(IN1, LOW);
//     digitalWrite(IN2, HIGH);
//     analogWrite(FNB, 100);
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);

//     float startYaw = yaw;
//     float targetYaw = startYaw + 160.0;

//     while (yaw < targetYaw) {
//         updateMPU();

//         delay(5);
//     }

//     stopMotors();
//     Serial.println(F("Rough 180 turn complete"));

//     currentBearing = newBearing;

//     for (int i = 0; i < 6; i++) {
//         alignToBearing(newBearing);
//     }
// }

void stopMotors() {
    BooleanFlags.isMovingForward = 0;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void 