#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <EEPROM.h>

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
#define TARGET_DISTANCE 25.0

// Motor calibration to fix right-side drift
#define LEFT_MOTOR_CALIBRATION 1.0
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

// extern char movement_arr[SIZE];
// extern uint8_t junction_nodes[SIZE];
// extern uint8_t junction_visited[SIZE];
// extern uint8_t index;
// extern uint8_t count;

char movement_arr[SIZE];
uint8_t junction_nodes[SIZE];
uint8_t junction_visited[SIZE];
uint8_t index;
uint8_t count;

struct BooleanFlags {
    uint8_t isMovingForward : 1;
    uint8_t targetReached : 1;
    uint8_t correctionEnabled : 1;
    uint8_t is_LeBron_done : 1;
    uint8_t has_LeBron_written : 1;
};

// extern BooleanFlags flags;
// extern EncoderState encoderState;
// extern MotorState motorState;
// extern MPUState mpuState;
// extern BearingState bearingState;

BooleanFlags flags;
EncoderState encoderState;
MotorState motorState;
MPUState mpuState;
BearingState bearingState;

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
void moveForwards(int PWM, MPUState &mpu, BearingState &bearing, MotorState &motor, EncoderState &encoder);
void Forward25(MPUState &mpu, BearingState &bearing, MotorState &motor, EncoderState &encoder);
void turn_left_90(MPUState &mpu, BearingState &bearing);
void turn_right_90(MPUState &mpu, BearingState &bearing);
void turn_180(MPUState &mpu, BearingState &bearing);
void stopMotors();
void updateDistance(EncoderState &encoder, MotorState &motor);
float getDistance(int trigPin, int echoPin);
int checkDist(int trigPin, int echoPin);

void leftEncoderISR() { encoderState.leftPulses++; }
void rightEncoderISR() { encoderState.rightPulses++; }

void ultrasonicSetup(int trigPin, int echoPin) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void calculateError(MPUState &mpu) {
    for (int i = 0; i < 1000; i++) {
        getOrientation(mpu);
        mpu.GyroErrorX += mpu.gyroX;
        mpu.GyroErrorY += mpu.gyroY;
        mpu.GyroErrorZ += mpu.gyroZ;
        delay(1);
    }
    mpu.GyroErrorX /= 1000;
    mpu.GyroErrorY /= 1000;
    mpu.GyroErrorZ /= 1000;
    Serial.println("Gyroscope calibration complete.");
}

void getOrientation(MPUState &mpu) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    mpu.gyroX = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    mpu.gyroY = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    mpu.gyroZ = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
}

void getAcceleration(MPUState &mpu) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    mpu.accelX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    mpu.accelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    mpu.accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

void updateMPU(MPUState &mpu) {
    mpu.previousTime = mpu.currentTime;
    mpu.currentTime = millis();
    mpu.elapsedTime = (mpu.currentTime - mpu.previousTime) * 0.001;
    
    getOrientation(mpu);
    getAcceleration(mpu);
    
    mpu.gyroX -= mpu.GyroErrorX;
    mpu.gyroY -= mpu.GyroErrorY;
    mpu.gyroZ -= mpu.GyroErrorZ;
    
    mpu.accelAngleX = atan2(mpu.accelY, sqrt(pow(mpu.accelX, 2) + pow(mpu.accelZ, 2))) * 180 / PI;
    mpu.accelAngleY = atan2(-mpu.accelX, sqrt(pow(mpu.accelY, 2) + pow(mpu.accelZ, 2))) * 180 / PI;
    
    mpu.roll = ALPHA * (mpu.roll + mpu.gyroX * mpu.elapsedTime) + (1 - ALPHA) * mpu.accelAngleX;
    mpu.pitch = ALPHA * (mpu.pitch + mpu.gyroY * mpu.elapsedTime) + (1 - ALPHA) * mpu.accelAngleY;
    mpu.yaw += mpu.gyroZ * mpu.elapsedTime; // Update yaw based on gyroZ
}

float normalizeYaw(float rawYaw) {
    float normalized = fmod(rawYaw, 360.0);
    if (normalized < 0) normalized += 360.0;
    return normalized;
}

float getCurrentRelativeBearing(const MPUState &mpu, const BearingState &bearing) {
    float relativeBearing = mpu.yaw - mpu.initialYaw;
    return normalizeYaw(relativeBearing);
}

void printCurrentBearing(const MPUState &mpu, const BearingState &bearing) {
    float relativeBearing = getCurrentRelativeBearing(mpu, bearing);
    Serial.print("Current Relative Bearing: ");
    Serial.print(relativeBearing);
    Serial.println("°");
}

void alignToBearing(MPUState &mpu, BearingState &bearing, float targetRelativeBearing) {
    float currentRelativeBearing = getCurrentRelativeBearing(mpu, bearing);
    float error = targetRelativeBearing - currentRelativeBearing;
    
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    if (abs(error) > BEARING_TOLERANCE) {
        if (error > 0) {
            analogWrite(FNA, 80 * LEFT_MOTOR_CALIBRATION);
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(FNB, 80 * RIGHT_MOTOR_CALIBRATION);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            Serial.println("Turning LEFT to align");
        } else {
            analogWrite(FNA, 80 * LEFT_MOTOR_CALIBRATION);
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(FNB, 80 * RIGHT_MOTOR_CALIBRATION);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            Serial.println("Turning RIGHT to align");
        }
        
        float currentError;
        do {
            updateMPU(mpu);
            currentRelativeBearing = getCurrentRelativeBearing(mpu, bearing);
            currentError = targetRelativeBearing - currentRelativeBearing;
            
            if (currentError > 180) currentError -= 360;
            if (currentError < -180) currentError += 360;
            
            Serial.print("Aligning - Current relative bearing: ");
            Serial.print(currentRelativeBearing);
            Serial.print("°, Error: ");
            Serial.println(currentError);
            
            delay(10);
        } while (abs(currentError) > BEARING_TOLERANCE * 0.5);
        
        stopMotors();
        Serial.println("Alignment complete");
        bearing.currentRelativeBearing = targetRelativeBearing;
    }
}

void maintainBearing(MPUState &mpu, BearingState &bearing, MotorState &motor) {
    if (!bearing.correctionEnabled) return;
    
    float currentRelative = getCurrentRelativeBearing(mpu, bearing);
    float error = bearing.currentRelativeBearing - currentRelative;
    
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    if (abs(error) > BEARING_TOLERANCE) {
        Serial.print("Bearing correction needed. Error: ");
        Serial.println(error);
        
        int correctionPWM = min(abs(error) * 2, 50);
        
        if (motor.isMovingForward) {
            if (error > 0) {
                analogWrite(FNA, (100 * LEFT_MOTOR_CALIBRATION) - correctionPWM);
                analogWrite(FNB, (100 * RIGHT_MOTOR_CALIBRATION) + correctionPWM);
            } else {
                analogWrite(FNA, (100 * LEFT_MOTOR_CALIBRATION) + correctionPWM);
                analogWrite(FNB, (100 * RIGHT_MOTOR_CALIBRATION) - correctionPWM);
            }
            
            delay(50);
            MoveForward(100);
        } else {
            alignToBearing(mpu, bearing, bearing.currentRelativeBearing);
        }
    }
}

void MoveForward(int PWM) {
    motorState.isMovingForward = true;
    analogWrite(FNA, PWM);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, PWM);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving Forward...");
}

void moveForwards(int PWM, MPUState &mpu, BearingState &bearing, MotorState &motor, EncoderState &encoder) {
    if (!motor.isMovingForward && !motor.targetReached) {
        motor.targetReached = false;
    }

    motor.isMovingForward = true;

    // Store the current absolute bearing at the start of movement
    float targetAbsoluteBearing = mpu.yaw; // Use the MPU's raw yaw as the absolute bearing

    Serial.print("Starting movement at absolute bearing: ");
    Serial.println(targetAbsoluteBearing);

    // Start moving forward
    analogWrite(FNA, PWM * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, PWM * RIGHT_MOTOR_CALIBRATION);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    Serial.println("Moving Forward...");

    unsigned long startTime = millis(); // Track the start time of the movement

    // Loop to maintain absolute bearing while moving forward
    while (motor.isMovingForward && !motor.targetReached) {
        updateMPU(mpu); // Update MPU data

        // Calculate the error between the target and current absolute bearing
        float error = targetAbsoluteBearing - mpu.yaw;

        // Normalize the error to the range [-180, 180]
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        Serial.print("The error is ");
        Serial.println(error);

        // If the error exceeds the tolerance, adjust motor speeds to correct the bearing
        if (abs(error) > BEARING_TOLERANCE && millis() - startTime < 2000) { // Only correct for 2 seconds
            int correctionPWM = min(abs(error) * 8, 50); // Proportional correction factor //change the value of abs(error) * VAR 

            if (error > 0) {
                // Turn slightly left (reduce left motor speed)
                analogWrite(FNA, (PWM * LEFT_MOTOR_CALIBRATION) - correctionPWM);
                analogWrite(FNB, (PWM * RIGHT_MOTOR_CALIBRATION) + correctionPWM);
            } else {
                // Turn slightly right (reduce right motor speed)
                analogWrite(FNA, (PWM * LEFT_MOTOR_CALIBRATION) + correctionPWM);
                analogWrite(FNB, (PWM * RIGHT_MOTOR_CALIBRATION) - correctionPWM);
            }

            Serial.print("Correcting Bearing - Error: ");
            Serial.println(error);
        } else {
            // Maintain straight movement if within tolerance or after 2 seconds
            analogWrite(FNA, PWM * LEFT_MOTOR_CALIBRATION);
            analogWrite(FNB, PWM * RIGHT_MOTOR_CALIBRATION);
        }

        // Update distance traveled
        updateDistance(encoder, motor);
        float avgDistance = (encoder.leftTotalDistance + encoder.rightTotalDistance) / 2.0;

        // Stop if the target distance is reached
        if (avgDistance >= TARGET_DISTANCE) {
            stopMotors();
            motor.targetReached = true;
            Serial.println("Target distance of 25 cm reached. Stopping.");
            break; // Exit the loop immediately
        }

        delay(10); // Small delay to avoid overloading the loop
    }
    maintainBearing(mpu, bearing, motor);

    // Stop motors when the target is reached or movement is interrupted
    stopMotors();
    Serial.println("Forward movement complete.");
}

void Forward25(MPUState &mpu, BearingState &bearing, MotorState &motor, EncoderState &encoder) {
    encoder.leftTotalDistance = 0.0;
    encoder.rightTotalDistance = 0.0;
    motor.targetReached = false;
    encoder.leftPulses = 0;
    encoder.rightPulses = 0;
    Serial.println("Distance tracking reset for forward movement");

    bearing.correctionEnabled = false;

    Serial.println("Starting forward movement...");

    moveForwards(100, mpu, bearing, motor, encoder);

    if (motor.targetReached) {
        Serial.println("Target distance reached.");
    }

    bearing.correctionEnabled = true;
}

void turn_left_90(MPUState &mpu, BearingState &bearing) {
    float newRelativeBearing = bearing.currentRelativeBearing + 90;
    if (newRelativeBearing >= 360) newRelativeBearing -= 360;
    
    Serial.print("Turning left 90° from relative bearing ");
    Serial.print(bearing.currentRelativeBearing);
    Serial.print("° to ");
    Serial.print(newRelativeBearing);
    Serial.println("°");
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(FNA, 255 * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, 255 * RIGHT_MOTOR_CALIBRATION);
    delay(15);
    analogWrite(FNA, 80 * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, 100 * RIGHT_MOTOR_CALIBRATION);
    
    float startYaw = mpu.yaw;
    float targetYaw = startYaw + 90.0;
    
    while (mpu.yaw < targetYaw) {
        updateMPU(mpu);
        Serial.print("Turning: Current Yaw = ");
        Serial.print(mpu.yaw);
        Serial.print(", Target = ");
        Serial.println(targetYaw);
        delay(1);
    }
    
    stopMotors();
    Serial.println("Rough turn complete");
    
    bearing.currentRelativeBearing = newRelativeBearing;
    alignToBearing(mpu, bearing, newRelativeBearing);
    
    unsigned long alignmentStartTime = millis();
    bool isAligned = false;
    
    while (!isAligned) {
        updateMPU(mpu);
        float currentRelative = getCurrentRelativeBearing(mpu, bearing);
        float error = newRelativeBearing - currentRelative;
        
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        
        if (abs(error) <= BEARING_TOLERANCE) {
            if (alignmentStartTime == 0) {
                alignmentStartTime = millis();
            }
            
            if (millis() - alignmentStartTime >= 500) {
                isAligned = true;
            }
        } else {
            alignmentStartTime = 0;
            maintainBearing(mpu, bearing, motorState);
        }
        
        Serial.print("Aligning - Current relative bearing: ");
        Serial.print(currentRelative);
        Serial.print("°, Error: ");
        Serial.println(error);
        
        delay(1);
    }
    
    stopMotors();
    Serial.println("Alignment complete. Robot is at target bearing.");
}

void turn_right_90(MPUState &mpu, BearingState &bearing) {
    float newRelativeBearing = bearing.currentRelativeBearing - 90;
    if (newRelativeBearing < 0) newRelativeBearing += 360;
    
    Serial.print("Turning right 90° from relative bearing ");
    Serial.print(bearing.currentRelativeBearing);
    Serial.print("° to ");
    Serial.print(newRelativeBearing);
    Serial.println("°");

    
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(FNA, 255 * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, 225 * RIGHT_MOTOR_CALIBRATION);
    delay(15);
    analogWrite(FNA, 100 * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, 80* RIGHT_MOTOR_CALIBRATION);
    
    float startYaw = mpu.yaw;
    float targetYaw = startYaw - 90.0;
    
    while (mpu.yaw > targetYaw) {
        updateMPU(mpu);
        Serial.print("Turning: Current Yaw = ");
        Serial.print(mpu.yaw);
        Serial.print(", Target = ");
        Serial.println(targetYaw);
        delay(1);
    }
    
    stopMotors();
    Serial.println("Rough turn complete");
    
    bearing.currentRelativeBearing = newRelativeBearing;
    alignToBearing(mpu, bearing, newRelativeBearing);
    
    unsigned long alignmentStartTime = millis();
    bool isAligned = false;
    
    while (!isAligned) {
        updateMPU(mpu);
        float currentRelative = getCurrentRelativeBearing(mpu, bearing);
        float error = newRelativeBearing - currentRelative;
        
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        
        if (abs(error) <= BEARING_TOLERANCE) {
            if (alignmentStartTime == 0) {
                alignmentStartTime = millis();
            }
            
            if (millis() - alignmentStartTime >= 500) {
                isAligned = true;
            }
        } else {
            alignmentStartTime = 0;
            maintainBearing(mpu, bearing, motorState);
        }
        
        Serial.print("Aligning - Current relative bearing: ");
        Serial.print(currentRelative);
        Serial.print("°, Error: ");
        Serial.println(error);
        
        delay(1);
    }
    
    stopMotors();
    Serial.println("Alignment complete. Robot is at target bearing.");
}

void turn_180(MPUState &mpu, BearingState &bearing) {
    float newRelativeBearing = bearing.currentRelativeBearing + 180;
    if (newRelativeBearing >= 360) newRelativeBearing -= 360;

    Serial.print("Turning 180° from relative bearing ");
    Serial.print(bearing.currentRelativeBearing);
    Serial.print("° to ");
    Serial.print(newRelativeBearing);
    Serial.println("°");

    analogWrite(FNA, 115 * LEFT_MOTOR_CALIBRATION);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 115 * RIGHT_MOTOR_CALIBRATION);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    float startYaw = mpu.yaw;
    float targetYaw = startYaw + 180.0;

    while (mpu.yaw < targetYaw) {
        updateMPU(mpu);
        Serial.print("Turning: Current Yaw = ");
        Serial.print(mpu.yaw);
        Serial.print(", Target = ");
        Serial.println(targetYaw);
        delay(1);
    }

    stopMotors();
    Serial.println("Rough 180° turn complete");

    bearing.currentRelativeBearing = newRelativeBearing;
    alignToBearing(mpu, bearing, newRelativeBearing);

    unsigned long alignmentStartTime = millis();
    bool isAligned = false;

    while (!isAligned) {
        updateMPU(mpu);
        float currentRelative = getCurrentRelativeBearing(mpu, bearing);
        float error = newRelativeBearing - currentRelative;
        
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        
        if (abs(error) <= BEARING_TOLERANCE) {
            if (alignmentStartTime == 0) {
                alignmentStartTime = millis();
            }
            
            if (millis() - alignmentStartTime >= 500) {
                isAligned = true;
            }
        } else {
            alignmentStartTime = 0;
            maintainBearing(mpu, bearing, motorState);
        }
        
        Serial.print("Aligning - Current relative bearing: ");
        Serial.print(currentRelative);
        Serial.print("°, Error: ");
        Serial.println(error);
        
        delay(1);
    }

    stopMotors();
    Serial.println("Alignment complete. Robot is at target bearing.");
}

void stopMotors() {
    motorState.isMovingForward = false;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stopped.");
}

void updateDistance(EncoderState &encoder, MotorState &motor) {
    if (motor.isMovingForward) {
        encoder.leftTotalDistance += (encoder.leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        encoder.rightTotalDistance += (encoder.rightPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        
        encoder.leftPulses = 0;
        encoder.rightPulses = 0;

        float avgDistance = (encoder.leftTotalDistance + encoder.rightTotalDistance) / 2.0;
        Serial.print("Avg Distance: ");
        Serial.println(avgDistance);

        if (avgDistance >= TARGET_DISTANCE && !motor.targetReached) {
            stopMotors();
            motor.targetReached = true;
        }
    } else {
        encoder.leftPulses = 0;
        encoder.rightPulses = 0;
    }
}

float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    Serial.print(F("NIAMA DISTANCE FOR TRIG"));
    Serial.println(trigPin);
    Serial.print(F("DISTANCE: "));
    Serial.println(duration * 0.0343 / 2.0);
    return (duration == 0) ? 0 : (duration * 0.0343 / 2.0);
}

//returns true or false depending on the distance from ultrasonic sensors
int checkDist(int trigPin, int echoPin) {
  float distance = getDistance(trigPin, echoPin);
    if (distance > 75) return -1; 
    else if (distance <= 10) return 0;
    else return 1;
}

void memoryReset() 
{
    Serial.println("Executing memoryReset()");
    for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);
    }
    Serial.println("EEPROM reset complete.");
}

void memoryWrite() 
{
    Serial.println("Executing memoryWrite()");
    int addr = 0;

    EEPROM.put(addr, movement_arr);
    addr += sizeof(movement_arr);

    EEPROM.put(addr, junction_nodes);
    addr += sizeof(junction_nodes);

    EEPROM.put(addr, index);

    Serial.println("EEPROM write complete.");
}

int memoryRead() 
{
    Serial.println("Executing memoryRead()");
    int addr = 0;
    if(EEPROM.read(0) != 0)
    {
        Serial.println("EEPROM is empty.");
        return -1;  // EEPROM is empty
    }

    EEPROM.get(addr, movement_arr);
    addr += sizeof(movement_arr);

    EEPROM.get(addr, junction_nodes);
    addr += sizeof(junction_nodes);

    EEPROM.get(addr, index);

    Serial.println("EEPROM read complete.");
    return 0;
}

void backtrack_and_reorient()
{
    Serial.println("backtrack_and_reorient starting.");
    turn_180(mpuState, bearingState);
    delay(5000);

    for (count -= 1; count > junction_nodes[index]; count--)
    {
        // Serial.print("Count: ");
        Serial.println(count);

        if (movement_arr[count] == 'F')
        {
            // Serial.println("Move forward now");
            Forward25(mpuState, bearingState, motorState, encoderState);
        }
        else if (movement_arr[count] == 'L')
        {
            // Serial.println("Turn right now");
            turn_right_90(mpuState, bearingState);
        }
        else if (movement_arr[count] == 'R')
        {
            // Serial.println("Turn left now");
            turn_left_90(mpuState, bearingState);
        }
    }

    // offset comepensation for front moevement from junction node only
    if (movement_arr[junction_nodes[index]] == 'F')
    {
        // Serial.println("Move forward now");
        Forward25(mpuState, bearingState, motorState, encoderState);
    }

    // Reorientation
    if (movement_arr[count] == 'F')
    {
        // Serial.println("Reorienting: F (Forward)");
        // Serial.println("Turn 180 now");
        turn_180(mpuState, bearingState);
        junction_visited[index] = 1;
        // Serial.println("Junction visited stored as 1.");
    }
    else if (movement_arr[count] == 'L')
    {
        // Serial.println("Reorienting: L (Left)");
        // Serial.println("Turn left now");
        turn_left_90(mpuState, bearingState);
        junction_visited[index] = 2;
        // Serial.println("Junction visited stored as 2.");
    }
    else if (movement_arr[count] == 'R')
    {
        // Serial.println("Reorienting: R (Right)");
        // Serial.println("Turn right now");
        turn_right_90(mpuState, bearingState);
        junction_visited[index] = 3;
        // Serial.println("Junction visited stored as 3.");
    }
}

void search_maze()
{
    Serial.print("Currently in loop: ");
    Serial.println(count);

    int front, left, right;

    front = checkDist(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    left = checkDist(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    right = checkDist(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

    Serial.print("Front: ");
    Serial.println(front);
    Serial.print("Left: ");
    Serial.println(left);
    Serial.print("Right: ");
    Serial.println(right);
    Serial.println("-----");
    Serial.println(movement_arr);
    Serial.println("-----");

    // Serial.println("-----");
    // Serial.print("Index: ");
    // Serial.println(index);
    // Serial.print("Junction visited: ");
    // Serial.println(junction_visited[index]);
    // Serial.println("-----");

    delay(5000);

    if ((front == -1 && left == -1) || (front == -1 && right == -1))
    {
        if (front == -1 && left == -1 && right == -1)
        {
            Serial.println("End of maze reached.");
            movement_arr[count] = '\0';
            // is_LeBron_done = true;
            flags.is_LeBron_done = 1;
            return;
        }
        return;
    }

    if (front != 0 && (count != junction_nodes[index] || junction_visited[index] < 1)) // front has space
    {
        // Serial.println("Front space detected");
        if (left == 1 || right == 1)
        {
            // Serial.println("Left and/or right space detected");
            index++;
            junction_nodes[index] = count;
            Serial.println("Junction node stored.");
        }

        // Serial.println("Move forward now.");
        Forward25(mpuState, bearingState, motorState, encoderState);
        // Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        // Serial.println("Forward movement stored.");
    }
    else if (left != 0 && (count != junction_nodes[index] || junction_visited[index] < 2)) // left has space
    {
        // Serial.println("Left space detected");
        if (right == 1)
        {
            // Serial.println("Right space detected");
            index++;

            junction_nodes[index] = count;
            // Serial.println(F("Junction node stored."));
        }

        // Serial.println("Turn left now");
        turn_left_90(mpuState, bearingState);
        // Serial.println("Turn left 90° done.");
        movement_arr[count] = 'L';
        count++;
        // Serial.println("Left turn stored.");

        // Serial.println("Move forward now.");
        Forward25(mpuState, bearingState, motorState, encoderState);
        // Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        // Serial.println("Forward movement stored.");
    }
    else if (right != 0 && (count != junction_nodes[index] || junction_visited[index] < 3)) // right has space
    {
        // Serial.println("Right space detected");

        // Serial.println("Turn right now");
        turn_right_90(mpuState, bearingState);
        // Serial.println("Turn right 90° done.");
        movement_arr[count] = 'R';
        count++;
        // Serial.println("Right turn stored.");

        // Serial.println("Move forward now.");
        Forward25(mpuState, bearingState, motorState, encoderState);
        // Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        // Serial.println("Forward movement stored.");
    }
    else
    {
        if ((junction_visited[index] == 2 && right == 0) || junction_visited[index] == 3)
        {
            Serial.println("All routes explored, removing junction.");
            junction_visited[index] = 0;
            index--;
        }

        backtrack_and_reorient();
        // Serial.println("backtrack_and_reorienting complete.");
    }
}

void init_arrays()
{
    // Serial.println("Initializing arrays.");

    memset(movement_arr, 0, sizeof(movement_arr));
    memset(junction_nodes, 0, sizeof(junction_nodes));
    memset(junction_visited, 0, sizeof(junction_visited));

    // memoryReset();
    // Serial.println("EEPROM reseted.");
}

void setup()
{
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
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    calculateError(mpuState);
    
    encoderState.leftTotalDistance = 0.0;
    encoderState.rightTotalDistance = 0.0;
    motorState.targetReached = false;

    delay(1000);
    updateMPU(mpuState);
    mpuState.initialYaw = mpuState.yaw;
    bearingState.currentRelativeBearing = 0.0;

    Serial.print("Initial yaw set to: ");
    Serial.println(mpuState.initialYaw);
    // Serial.println("Starting with relative bearing of 0 degrees");

    init_arrays();
}

void loop()
{
    static unsigned long lastPrintTime = 0;
    const unsigned long printInterval = 500;

    updateMPU(mpuState);

    if (millis() - lastPrintTime >= printInterval) {
        printCurrentBearing(mpuState, bearingState);
        lastPrintTime = millis();
    }

    if (flags.is_LeBron_done == 0) 
    {
        search_maze();
    }
    else
    {
        if (flags.has_LeBron_written == 0)
        {
            memoryWrite();
            flags.has_LeBron_written = true;
        }
    }
}