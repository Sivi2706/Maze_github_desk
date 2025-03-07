#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <EEPROM.h>

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
#define TARGET_DISTANCE 25.0

// Motor calibration to fix right-side drift
#define LEFT_MOTOR_CALIBRATION 1.0
#define RIGHT_MOTOR_CALIBRATION 0.95

// Encoder and distance calculation variables
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;
const unsigned int PULSES_PER_TURN = 20;
const float WHEEL_DIAMETER = 4;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Distance tracking
float leftTotalDistance = 0.0;
float rightTotalDistance = 0.0;
bool isMovingForward = false;
bool targetReached = false;

// Bearing system variables
#define NORTH 0
#define EAST 90
#define SOUTH 180
#define WEST 270
int currentBearing = NORTH;
bool correctionEnabled = true;

// Correction timing variables
unsigned long lastCorrectionTime = 0;
const unsigned long CORRECTION_INTERVAL = 500;

// Interrupt service routines
void leftEncoderISR() { leftPulses++; }
void rightEncoderISR() { rightPulses++; }

bool hasStarted = false;
float initialDistance = 0;
float totalDistance = 0;

// RPM calculation variables
unsigned long lastLeftPulseTime = 0;
unsigned long lastRightPulseTime = 0;
float leftRPM = 0.0;
float rightRPM = 0.0;

// Row and Column for maze
#define ROWS 8
#define COLS 8
#define MAX_GOONS ROWS * COLS

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

inline int checkTheDih(int trigPin, int echoPin) {
    if (getDistance(trigPin, echoPin) > 200) return -1;
    return getDistance(trigPin, echoPin) <= 10 ? 0 : 1;
}

// MPU6050 Setup
const int MPU = 0x68;
const float GYRO_SCALE = 1.0 / 131.0;
float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;
float accelAngleX, accelAngleY;
float roll, pitch, yaw;
float GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, previousTime, currentTime;
float initialYaw = 0.0;
const float ALPHA = 0.96;

void getOrientation() {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    gyroX = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    gyroY = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
    gyroZ = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
}

void getAcceleration() {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    accelX = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accelY = (Wire.read() << 8 | Wire.read()) / 16384.0;
    accelZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

void calculateError() {
    for (int i = 0; i < 1000; i++) {
        getOrientation();
        GyroErrorX += gyroX;
        GyroErrorY += gyroY;
        GyroErrorZ += gyroZ;
        delay(1);
    }
    GyroErrorX /= 1000;
    GyroErrorY /= 1000;
    GyroErrorZ /= 1000;
    Serial.println("Gyroscope calibration complete.");
}

void updateMPU() {
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) * 0.001;
    
    getOrientation();
    getAcceleration();
    
    gyroX -= GyroErrorX;
    gyroY -= GyroErrorY;
    gyroZ -= GyroErrorZ;
    
    accelAngleX = atan2(accelY, sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI;
    accelAngleY = atan2(-accelX, sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI;
    
    roll = ALPHA * (roll + gyroX * elapsedTime) + (1 - ALPHA) * accelAngleX;
    pitch = ALPHA * (pitch + gyroY * elapsedTime) + (1 - ALPHA) * accelAngleY;
    yaw += gyroZ * elapsedTime;
}

float normalizeYaw(float rawYaw) {
    float normalized = fmod(rawYaw, 360.0);
    if (normalized < 0) normalized += 360.0;
    return normalized;
}

int getCurrentAbsoluteBearing() {
    float relativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(relativeYaw);
    int absoluteBearing = round(normalizedYaw / 90) * 90;
    if (absoluteBearing >= 360) absoluteBearing = 0;
    return absoluteBearing;
}

void turnRight90() {
    int newBearing = currentBearing - 90;
    if (newBearing < 0) newBearing += 360;
    
    Serial.print("Turning right 90° from bearing ");
    Serial.print(currentBearing);
    Serial.print("° to ");
    Serial.print(newBearing);
    Serial.println("°");
    
    analogWrite(FNA, 100 * LEFT_MOTOR_CALIBRATION);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, 100 * RIGHT_MOTOR_CALIBRATION);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    
    float startYaw = yaw;
    float targetYaw = startYaw - 80.0;
    
    while (yaw > targetYaw) {
        updateMPU();
        Serial.print("Turning: Current Yaw = ");
        Serial.print(yaw);
        Serial.print(", Target = ");
        Serial.println(targetYaw);
        delay(10);
    }
    
    stopMotors();
    Serial.println("Rough turn complete");
    
    currentBearing = newBearing;
    alignToBearing(newBearing);
}

void turnLeft90() {
    int newBearing = currentBearing + 90;
    if (newBearing >= 360) newBearing -= 360;
    
    Serial.print("Turning left 90° from bearing ");
    Serial.print(currentBearing);
    Serial.print("° to ");
    Serial.print(newBearing);
    Serial.println("°");
    
    analogWrite(FNA, 130 * LEFT_MOTOR_CALIBRATION);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 130 * RIGHT_MOTOR_CALIBRATION);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    float startYaw = yaw;
    float targetYaw = startYaw + 80.0;
    
    while (yaw < targetYaw) {
        updateMPU();
        Serial.print("Turning: Current Yaw = ");
        Serial.print(yaw);
        Serial.print(", Target = ");
        Serial.println(targetYaw);
        delay(10);
    }
    
    stopMotors();
    Serial.println("Rough turn complete");
    
    currentBearing = newBearing;
    alignToBearing(newBearing);
}

void turn180() {
    int newBearing = currentBearing + 180;
    if (newBearing >= 360) newBearing -= 360;

    Serial.print("Turning 180° from bearing ");
    Serial.print(currentBearing);
    Serial.print("° to ");
    Serial.print(newBearing);
    Serial.println("°");

    analogWrite(FNA, 100 * LEFT_MOTOR_CALIBRATION);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 100 * RIGHT_MOTOR_CALIBRATION);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    float startYaw = yaw;
    float targetYaw = startYaw + 170.0;

    while (yaw < targetYaw) {
        updateMPU();
        Serial.print("Turning: Current Yaw = ");
        Serial.print(yaw);
        Serial.print(", Target = ");
        Serial.println(targetYaw);
        delay(10);
    }

    stopMotors();
    Serial.println("Rough 180° turn complete");

    currentBearing = newBearing;
    alignToBearing(newBearing);
}

void alignToBearing(int targetBearing) {
    int currentAbsoluteBearing = getCurrentAbsoluteBearing();
    float currentRelativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(currentRelativeYaw);
    
    float error = targetBearing - normalizedYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    Serial.print("Aligning to bearing: ");
    Serial.print(targetBearing);
    Serial.print("°, Current normalized yaw: ");
    Serial.print(normalizedYaw);
    Serial.print("°, Error: ");
    Serial.println(error);
    
    const float BEARING_TOLERANCE = 2.0;
    
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
            updateMPU();
            currentRelativeYaw = yaw - initialYaw;
            normalizedYaw = normalizeYaw(currentRelativeYaw);
            currentError = targetBearing - normalizedYaw;
            if (currentError > 180) currentError -= 360;
            if (currentError < -180) currentError += 360;
            Serial.print("Aligning - Current yaw: ");
            Serial.print(normalizedYaw);
            Serial.print("°, Error: ");
            Serial.println(currentError);
            delay(10);
        } while (abs(currentError) > BEARING_TOLERANCE * 0.5);
        
        stopMotors();
        Serial.println("Alignment complete");
        currentBearing = targetBearing;
    }
}

void maintainBearing() {
    if (!correctionEnabled) return;
    
    float currentRelativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(currentRelativeYaw);
    
    float error = currentBearing - normalizedYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    const float BEARING_TOLERANCE = 3.0;
    
    if (abs(error) > BEARING_TOLERANCE) {
        Serial.print("Bearing correction needed. Error: ");
        Serial.println(error);
        
        int correctionPWM = min(abs(error) * 2, 50);
        
        if (isMovingForward) {
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
            alignToBearing(currentBearing);
        }
    }
}

void MoveForward(int PWM) {
    isMovingForward = true;
    analogWrite(FNA, PWM);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, PWM);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving Forward...");
}

void moveForwards(int PWM) {
    if (!isMovingForward && !targetReached) {
        leftTotalDistance = 0.0;
        rightTotalDistance = 0.0;
        targetReached = false;
    }

    isMovingForward = true;
    analogWrite(FNA, PWM * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, PWM * RIGHT_MOTOR_CALIBRATION);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    while (isMovingForward && !targetReached) {
        updateMPU();
        float correction = initialYaw - yaw;

        if (correction > 3) {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else if (correction < -3) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        } else {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        }

        updateDistance();

        float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;
        if (avgDistance >= TARGET_DISTANCE && !targetReached) {
            stopMotors();
            targetReached = true;
            break;
        }

        delay(5);
    }
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

// Algorithm Functions
int back_it_up_LeBron(int back_index, char temp_movement[], int movement_index[], int* index_counter) {
    turn180();
    for (int i = back_index; i > movement_index[*index_counter]; i--) {
        if (temp_movement[i] == 'F') {
            moveForwards(100);
        } else if (temp_movement[i] == 'L') {
            turnRight90();
        } else if (temp_movement[i] == 'R') {
            turnLeft90();
        }
    }
    return movement_index[*index_counter];
}

void keep_going_LeBron(char temp_movement[], int movement_index[], int junction_gooned[], int* index_counter, int* i) {
    int front, left, right;
    for (; *i < MAX_GOONS; (*i)++) {
        front = checkTheDih(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
        left = checkTheDih(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
        right = checkTheDih(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

        if (front == -1 && left == -1 && right == -1) {
            temp_movement[*i] = '\0';
            Serial.println("chaochibai");
            break;
        } else if (junction_gooned[*index_counter] < 1 && front) {
            if (left || right) movement_index[++(*index_counter)] = *i;
            moveForwards(255);
            Serial.println("niama");
            temp_movement[*i] = 'F';
        } else if (junction_gooned[*index_counter] < 2 && left) {
            if (right) movement_index[++(*index_counter)] = *i;
            turnLeft90();
            Serial.println("ganninia")
            temp_movement[*i] = 'L';
            moveForwards(255);
            temp_movement[++(*i)] = 'F';
        } else if (junction_gooned[*index_counter] < 3 && right) {
            turnRight90();
            Serial.println("sohai");
            temp_movement[*i] = 'R';
            moveForwards(255);
            temp_movement[++(*i)] = 'F';
        } else if ((junction_gooned[*index_counter] == 2 && !right) || junction_gooned[*index_counter] == 3) {
            junction_gooned[*index_counter--] = 0;
            *i = back_it_up_LeBron(*i, temp_movement, movement_index, index_counter);
            if (temp_movement[*i + 1] == 'F') {
                turn180();
                Serial.println("chibai180");
                junction_gooned[*index_counter] = 1;
            } else if (temp_movement[*i + 1] == 'L') {
                turnLeft90();
                Serial.println("mamameiyoujiaoleft");
                junction_gooned[*index_counter] = 2;
            } else if (temp_movement[*i + 1] == 'R') {
                turnRight90();
                Serial.println("mf");
                junction_gooned[*index_counter] = 3;
            }
        } else {
            *i = back_it_up_LeBron(*i, temp_movement, movement_index, index_counter);
            if (temp_movement[*i + 1] == 'F') {
                turn180();
                Serial.println("ganninia180");
                junction_gooned[*index_counter] = 1;
            } else if (temp_movement[*i + 1] == 'L') {
                turnLeft90();
                Serial.println("sohaileft");
                junction_gooned[*index_counter] = 2;
            } else if (temp_movement[*i + 1] == 'R') {
                turnRight90();
                Serial.println("chibairight");
                junction_gooned[*index_counter] = 3;
            }
        }
    }
}

int follow_gooning_path(char temp_movement[], int movement_index[], int* index_counter) {
    int i;
    for (i = 0; i < movement_index[*index_counter - 1] + 1; i++) {
        if (temp_movement[i] == 'F') {
            moveForwards(100);
        } else if (temp_movement[i] == 'L') {
            turnLeft90();
            moveForwards(100);
        } else if (temp_movement[i] == 'R') {
            turnRight90();
            moveForwards(100);
        }
    }
    return i + 1;
}

void memoryReset() {
    for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0xFF);
    }
}

void memoryWrite(char final_movement[], int movement_index[], int* index_counter) {
    int addr = 0;
    for (int i = 0; i < MAX_GOONS; i++) {
        EEPROM.write(addr + i, final_movement[i]);
    }
    addr += MAX_GOONS;
    for (int i = 0; i < MAX_GOONS; i++) {
        EEPROM.put(addr + i * sizeof(int), movement_index[i]);
    }
    addr += MAX_GOONS * sizeof(int);
    EEPROM.put(addr, *index_counter);
}

int memoryRead(char final_movement[], int movement_index[], int* index_counter) {
    int addr = 0;
    byte check = EEPROM.read(addr);
    if (check == 0xFF) {
        return -1;
    }
    for (int i = 0; i < MAX_GOONS; i++) {
        final_movement[i] = EEPROM.read(addr + i);
    }
    addr += MAX_GOONS;
    for (int i = 0; i < MAX_GOONS; i++) {
        EEPROM.get(addr + i * sizeof(int), movement_index[i]);
    }
    addr += MAX_GOONS * sizeof(int);
    EEPROM.get(addr, *index_counter);
    return 0;
}

// Custom string length function
int myStrlen(const char* str) {
    int len = 0;
    while (str[len] != '\0') {
        len++;
    }
    return len;
}

void start_gooning() {
    char temp_movement[MAX_GOONS];
    char final_movement[MAX_GOONS];
    int movement_index[MAX_GOONS];
    int junction_gooned[MAX_GOONS];

    // Initialize arrays without memset
    for (int i = 0; i < MAX_GOONS; i++) {
        temp_movement[i] = 0;
        final_movement[i] = 0;
        movement_index[i] = 0;
        junction_gooned[i] = 0;
    }

    int index_counter = 0;
    int aura = 0;

    keep_going_LeBron(temp_movement, movement_index, junction_gooned, &index_counter, &aura);


    /*
    if (memoryRead(final_movement, movement_index, &index_counter) < 0) {
        keep_going_LeBron(temp_movement, movement_index, junction_gooned, &index_counter, &aura);
    } else {
        // Copy array without memcpy
        for (int i = 0; i < MAX_GOONS; i++) {
            final_movement[i] = temp_movement[i];
        }
        aura = follow_gooning_path(final_movement, movement_index, &index_counter);
        keep_going_LeBron(temp_movement, movement_index, junction_gooned, &index_counter, &aura);

        // Compare lengths without strlen
        if (myStrlen(temp_movement) < myStrlen(final_movement)) {
            memoryReset();
            memoryWrite(temp_movement, movement_index, &index_counter);
        }
    }
    */
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
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    calculateError();
    
    leftTotalDistance = 0.0;
    rightTotalDistance = 0.0;
    targetReached = false;

    delay(1000);
    updateMPU();
    initialYaw = yaw;
    Serial.print("Initial yaw set to: ");
    Serial.println(initialYaw);
    Serial.print("Starting with bearing: ");
    Serial.println(currentBearing);
}

bool done = false;

void loop() {
    start_gooning();
}