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

// Initialise global variables
char temp_movement[MAX_GOONS];
char final_movement[MAX_GOONS];
int movement_index[MAX_GOONS];
int junction_gooned[MAX_GOONS];
int index_counter = 0;
int aura = 0;

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

inline int rizzCheck(int trigPin, int echoPin) {
    float distance = getDistance(trigPin, echoPin);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    return distance <= 10 ? 0 : 1; // Threshold distance for junction detection
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

void getOrientation();
void getAcceleration();
void calculateError();
void updateMPU();
float normalizeYaw(float rawYaw);
int getCurrentAbsoluteBearing();
void stopMotors();
void alignToBearing(int targetBearing);
void turnRight90();
void turnLeft90();
void turn180();
void MoveForward(int PWM);
void maintainBearing();
void updateDistance(); 


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

void stopMotors() {
    isMovingForward = false;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stopped.");
}

void alignToBearing(int targetBearing) {
    Serial.println("Aligning...");
    float currentRelativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(currentRelativeYaw);
    
    float error = targetBearing - normalizedYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
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
    // Assume alignToBearing smoothly finalizes the turn
    // (see alignToBearing function below)
    // This call can help refine the turn if needed.
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
        delay(10);
    }

    stopMotors();
    Serial.println("Rough 180° turn complete");

    currentBearing = newBearing;
    alignToBearing(newBearing);
}

// Updated MoveForward: always reset distance tracking for each new forward move.
void MoveForward(int PWM) {
    // Reset distances and flag for the new movement segment
    leftTotalDistance = 0.0;
    rightTotalDistance = 0.0;
    targetReached = false;
    
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
        if (avgDistance >= TARGET_DISTANCE) {
            stopMotors();
            targetReached = true;
            break;
        }
        
        delay(5);
    }
}

void maintainBearing() {
    Serial.println("Maintaining bearing");
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

void updateDistance() {
    if (isMovingForward) {
        leftTotalDistance += (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        rightTotalDistance += (rightPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;
        Serial.print("Avg Distance: ");
        Serial.println(avgDistance);
        leftPulses = rightPulses = 0;
        if (avgDistance >= TARGET_DISTANCE) {
            stopMotors();
            targetReached = true;
        }
    }
}

// Maze Navigation Algorithm
int back_it_up_LeBron(int back_index, char* temp_movement, int* movement_index, int* index_counter) 
{
    Serial.println("Executing back_it_up_LeBron()");
    turn180();  // Turn away from the wall

    // Backtrack to the last junction
    for (int i = back_index; i > movement_index[*index_counter]; i--) 
    {
        Serial.print("Backtracking step: ");
        Serial.println(temp_movement[i]);
        if (temp_movement[i] == 'F') 
        {
            MoveForward(100);
        } 
        else if (temp_movement[i] == 'L') 
        {
            turnRight90();  // Opposite direction
        } 
        else if (temp_movement[i] == 'R') 
        {
            turnLeft90();   // Opposite direction
        }
    }
    // Return to the most recent junction
    Serial.println("Backtracking complete.");
    return movement_index[*index_counter];
}

void keep_going_LeBron(char* temp_movement, int* movement_index, int* junction_gooned, int* index_counter, int i) 
{
    Serial.println("Executing keep_going_LeBron()");
    int front, left, right;

    for (i = i; i < MAX_GOONS; i++) 
    {
        front = rizzCheck(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
        left = rizzCheck(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
        right = rizzCheck(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

        if (front == -1 && left == -1 && right == -1)                       // End of maze
        {
            Serial.println("End of maze reached.");
            temp_movement[i] = '\0';  // Null terminate the array
            break;
        } 
        else if (junction_gooned[*index_counter] < 1 && front)              // Forward path
        {
            if (left || right) movement_index[++(*index_counter)] = i;
            MoveForward(255);
            temp_movement[i] = 'F';
            Serial.println("Movement: F (Forward)");
        } 
        else if (junction_gooned[*index_counter] < 2 && left)               // Left path
        {
            if (right) movement_index[++(*index_counter)] = i;
            turnLeft90();
            temp_movement[i] = 'L';
            MoveForward(255);
            temp_movement[i++] = 'F';
            Serial.println("Movement: L (Left)");
        } 
        else if (junction_gooned[*index_counter] < 3 && right)              // Right path
        {
            turnRight90();
            temp_movement[i] = 'R';
            MoveForward(255);
            temp_movement[i++] = 'F';
            Serial.println("Movement: R (Right)");
        }
        // if all routes of the junction have been explored and are dead ends, then backtrack
        else if (junction_gooned[*index_counter] == 3 || (junction_gooned[*index_counter] == 2 && !right)) 
        {
            junction_gooned[(*index_counter)--] = 0;
            i = back_it_up_LeBron(i, temp_movement, movement_index, index_counter);
            
            if (temp_movement[i + 1] == 'F') 
            {
                turn180();
                Serial.println("Turning 180");
                Serial.println("Reorienting forward");
                junction_gooned[*index_counter] = 1;
            } 
            else if (temp_movement[i + 1] == 'L') 
            {
                turnLeft90();
                Serial.println("Turning 90 left");
                Serial.println("Reorienting left");
                junction_gooned[*index_counter] = 2;
            } 
            else if (temp_movement[i + 1] == 'R') 
            {
                turnRight90();
                Serial.println("Turning 90 right");
                Serial.println("Reorienting right");
                junction_gooned[*index_counter] = 3;
            }
        } 
        else 
        {
            Serial.println("Dead end detected. Backtracking...");
            i = back_it_up_LeBron(i, temp_movement, movement_index, index_counter);

            if (temp_movement[i + 1] == 'F') 
            {
                turn180();
                junction_gooned[*index_counter] = 1;
                Serial.println("Reorienting: F (Forward)");
            } 
            else if (temp_movement[i + 1] == 'L') 
            {
                turnLeft90();
                junction_gooned[*index_counter] = 2;
                Serial.println("Reorienting: L (Left)");
            } 
            else if (temp_movement[i + 1] == 'R') 
            {
                turnRight90();
                junction_gooned[*index_counter] = 3;
                Serial.println("Reorienting: R (Right)");
            }
        }
    }
}

int follow_Lebrons_footsteps(char* temp_movement, int* movement_index, int* index_counter) 
{
    Serial.println("Executing follow_Lebrons_footsteps()");
    int i;
    for (i = 0; i < movement_index[*index_counter - 1] + 1; i++) 
    {
        Serial.print("Following path step: ");
        Serial.println(temp_movement[i]);
        if (temp_movement[i] == 'F') 
        {
            MoveForward(100);
        } 
        else if (temp_movement[i] == 'L') 
        {
            turnLeft90();
            MoveForward(100);
        } 
        else if (temp_movement[i] == 'R') 
        {
            turnRight90();
            MoveForward(100);
        }
    }
    return i + 1;
}

// EEPROM Memory Functions
void memoryReset() {
    Serial.println("Executing memoryReset()");
    for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);
    }
    Serial.println("EEPROM reset complete.");
}

void memoryWrite(char* final_movement, int* movement_index, int* index_counter) {
    Serial.println("Executing memoryWrite()");
    int addr = 0;
    EEPROM.put(addr, final_movement);
    addr += sizeof(final_movement);
    EEPROM.put(addr, movement_index);
    addr += sizeof(movement_index);
    EEPROM.put(addr, *index_counter);
    Serial.println("EEPROM write complete.");
}

int memoryRead(char* final_movement, int* movement_index, int* index_counter) {
    Serial.println("Executing memoryRead()");
    int addr = 0;
    int check = 0;

    if (EEPROM.get(addr, check) == 0xFFFF) {
        Serial.println("EEPROM is empty.");
        return -1;  // EEPROM is empty
    }

    EEPROM.get(addr, final_movement);
    addr += sizeof(final_movement);
    EEPROM.get(addr, movement_index);
    addr += sizeof(movement_index);
    EEPROM.get(addr, *index_counter);

    Serial.println("EEPROM read complete.");
    return 0;
}

void this_time_i_want_youyouyouyou_like_its_magnetic() 
{
    Serial.println("this_time_i_want_youyouyouyou_like_its_magnetic");
    // char temp_movement[MAX_GOONS];
    // char final_movement[MAX_GOONS];
    // int movement_index[MAX_GOONS];
    // int junction_gooned[MAX_GOONS];
    // int index_counter = 0;
    // int aura = 0;

    memset(temp_movement, 0, sizeof(temp_movement));
    memset(final_movement, 0, sizeof(final_movement));
    memset(movement_index, 0, sizeof(movement_index));
    memset(junction_gooned, 0, sizeof(junction_gooned));

    // keep_going_LeBron(temp_movement, movement_index, junction_gooned, &index_counter, aura);

    // if (memoryRead(final_movement, movement_index, &index_counter) < 0) 
    // {
    //     Serial.println("No saved path found. Exploring new path...");
    //     keep_going_LeBron(temp_movement, movement_index, junction_gooned, &index_counter, aura);
    // }
    // else 
    // {
    //     Serial.println("Saved path found. Following path...");
    //     memcpy(final_movement, temp_movement, sizeof(final_movement));
    //     aura = follow_Lebrons_footsteps(final_movement, movement_index, &index_counter);
    //     keep_going_LeBron(temp_movement, movement_index, junction_gooned, &index_counter, aura);

    //     if (strlen(temp_movement) < strlen(final_movement)) 
    //     {
    //         Serial.println("New shorter path found. Updating EEPROM...");
    //         memoryReset();
    //         memoryWrite(temp_movement, movement_index, &index_counter);
    //     }
    // }
}

void setup() {
    Serial.begin(115200);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(FNA, OUTPUT);
    pinMode(FNB, OUTPUT);
    ultrasonicSetup(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    ultrasonicSetup(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    ultrasonicSetup(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);
    Serial.println("Setup complete.");
    this_time_i_want_youyouyouyou_like_its_magnetic();
}

void loop() 
{
    Serial.println("Executing keep_going_LeBron()");
    int front, left, right, i = 0;

    front = rizzCheck(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    left = rizzCheck(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    right = rizzCheck(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

    if (front == -1 && left == -1 && right == -1)                       // End of maze
    {
        Serial.println("End of maze reached.");
        temp_movement[i] = '\0';  // Null terminate the array
        // break;
    } 
    else if (junction_gooned[index_counter] < 1 && front)              // Forward path
    {
        if (left || right) movement_index[++index_counter] = i;
        MoveForward(100);
        temp_movement[i] = 'F';
        Serial.println("Movement: F (Forward)");
    } 
    else if (junction_gooned[index_counter] < 2 && left)               // Left path
    {
        if (right) movement_index[++index_counter] = i;
        turnLeft90();
        temp_movement[i] = 'L';
        MoveForward(100);
        temp_movement[i++] = 'F';
        Serial.println("Movement: L (Left)");
    } 
    else if (junction_gooned[index_counter] < 3 && right)              // Right path
    {
        turnRight90();
        temp_movement[i] = 'R';
        MoveForward(100);
        temp_movement[i++] = 'F';
        Serial.println("Movement: R (Right)");
    }
    // if all routes of the junction have been explored and are dead ends, then backtrack
    else if (junction_gooned[index_counter] == 3 || (junction_gooned[index_counter] == 2 && !right)) 
    {
        junction_gooned[(index_counter)--] = 0;
        i = back_it_up_LeBron(i, temp_movement, movement_index, &index_counter);
        
        if (temp_movement[i + 1] == 'F') 
        {
            turn180();
            Serial.println("Turning 180");
            Serial.println("Reorienting forward");
            junction_gooned[index_counter] = 1;
        } 
        else if (temp_movement[i + 1] == 'L') 
        {
            turnLeft90();
            Serial.println("Turning 90 left");
            Serial.println("Reorienting left");
            junction_gooned[index_counter] = 2;
        } 
        else if (temp_movement[i + 1] == 'R') 
        {
            turnRight90();
            Serial.println("Turning 90 right");
            Serial.println("Reorienting right");
            junction_gooned[index_counter] = 3;
        }
    } 
    else 
    {
        Serial.println("Dead end detected. Backtracking...");
        i = back_it_up_LeBron(i, temp_movement, movement_index, &index_counter);

        if (temp_movement[i + 1] == 'F') 
        {
            turn180();
            junction_gooned[index_counter] = 1;
            Serial.println("Reorienting: F (Forward)");
        } 
        else if (temp_movement[i + 1] == 'L') 
        {
            turnLeft90();
            junction_gooned[index_counter] = 2;
            Serial.println("Reorienting: L (Left)");
        } 
        else if (temp_movement[i + 1] == 'R') 
        {
            turnRight90();
            junction_gooned[index_counter] = 3;
            Serial.println("Reorienting: R (Right)");
        }
    }
}