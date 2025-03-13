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
#define SIZE ROWS * COLS

//Target distance to travel
#define TARGET_DISTANCE 25

//Calibrates the drifting of both motors (could be removed since we already use MPU for aligning)
#define LEFT_MOTOR_CALIBRATION 1.0
#define RIGHT_MOTOR_CALIBRATION 0.95

//Variables for encoders
int PULSES_PER_TURN = 20;
float WHEEL_CIRCUMFERENCE = PI * 4;

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


char movement_arr[SIZE];
uint8_t junction_nodes[SIZE];
uint8_t junction_visited[SIZE];
uint8_t index = 0;
uint8_t count = 0;
// bool is_LeBron_done = false;
// bool has_LeBron_written = false;

//Flags to store bools
struct BooleanFlags {
    unsigned int isMovingForward : 1;
    unsigned int targetReached : 1;
    unsigned int correctionEnabled : 1;
    unsigned int is_LeBron_done : 1;
    unsigned int has_LeBron_written : 1;
} flags = {0, 0, 1, 0, 0};

// struct EncoderState {
//     unsigned int leftPulses = 0;
//     unsigned int rightPulses = 0;
//     float leftTotalDistance = 0.0;
//     float rightTotalDistance = 0.0;
// };

unsigned int leftPulses = 0;
unsigned int rightPulses = 0;

// function prototypes
void stopMotors();
void moveForward(int PWM);

//ISP for rotary encoders
void leftEncoderISP() {leftPulses++;}
void rightEncoderISP() {rightPulses++;}

//setup ultrasonic sensors in initial setup() function
void ultrasonicSetup(int trigPin, int echoPin) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

//returns distance using ultrasonic sensors
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
    if (distance > 210) return -1; 
    else if (distance <= 10) return 0;
    else return 1;
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

    yaw += gyroZ * elapsedTime;
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
    if (flags.correctionEnabled == 0) return;

    float currentRelativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(currentRelativeYaw);

    float error = currentBearing - normalizedYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    const float BEARING_TOLERANCE = 3.0;

    if (abs(error) > BEARING_TOLERANCE) {
        int correctionPWM = min(abs(error) * 2, 50);

        if (flags.isMovingForward == 1) {
            if (error > 0) {
                analogWrite(FNA, 100 - correctionPWM);
                analogWrite(FNB, 100 - correctionPWM);
            } else {
                analogWrite(FNA, 100 + correctionPWM);
                analogWrite(FNB, 100 - correctionPWM);
            }

            delay(50);

            moveForward(100);
        } else {
            alignToBearing(currentBearing);
        }
    }
}

//updates the distance travelled
void updateDistance() {
    if (flags.isMovingForward) {
        leftTotalDistance += (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        rightTotalDistance += (rightPulses / (float)PULSES_PER_TURN) *WHEEL_CIRCUMFERENCE;
        float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;
        Serial.print(F("Avg distance: "));
        Serial.println(avgDistance);
        leftPulses = rightPulses = 0;
        if (avgDistance >= TARGET_DISTANCE && flags.targetReached == 0) {
            stopMotors();
            flags.targetReached = 1;
        }
    }
}

//stops all motors from moving 
void stopMotors() {
    flags.isMovingForward = 0;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

//moves forward until target distance, while following the bearing
void moveForward(int PWM) {
    flags.isMovingForward = 1;

    float targetBearing = currentBearing;

    analogWrite(FNA, PWM);
    analogWrite(FNB, PWM);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    Serial.println(F("Moving forward"));

    unsigned long startTime = millis();

    float BEARING_TOLERANCE = 2.0;

    while (flags.isMovingForward == 1 && flags.targetReached == 0) {
        updateMPU();

        float currentRelativeBearing = getCurrentAbsoluteBearing();

        float error = targetBearing - currentRelativeBearing;

        if (error > 180) error -= 360;
        if (error < - 180) error += 360;

        if (abs(error) > BEARING_TOLERANCE && millis() - startTime < 2000) {
            int correctionPWM = min(abs(error) * 2, 50);

            if (error > 0) {
                analogWrite(FNA, PWM - correctionPWM);
                analogWrite(FNB, PWM + correctionPWM);
            } else {
                analogWrite(FNA, PWM + correctionPWM);
                analogWrite(FNB, PWM - correctionPWM);
            }

            Serial.print(F("Correcting Bearing - Error: "));
            Serial.println(error);
        } else {
            analogWrite(FNA, PWM);
            analogWrite(FNB, PWM);
        }

        updateDistance();
        float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;

        if (avgDistance >= TARGET_DISTANCE) {
            stopMotors();
            flags.targetReached = 1;
            Serial.println(F("Target reached!"));
            break;
        }

        delay(5);
    }

    stopMotors();
    Serial.println(F("Forward completed"));
}

//turns left 90 degrees
void turn_left_90() {
    float targetBearing = currentBearing + 90;
    if (targetBearing >= 360) targetBearing -= 360;

    Serial.print(F("Turning left 90 to "));
    Serial.println(targetBearing);

    analogWrite(FNA, 150);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 150);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    float startYaw = yaw;
    float targetYaw = yaw + 80.0;

    while (yaw < targetYaw) {
        updateMPU();
        
        delay(5);
    }

    stopMotors();
    Serial.println(F("Rough turn completed"));

    currentBearing = targetBearing;
    alignToBearing(targetBearing);

    unsigned long alignmentStartTime = millis();
    bool isAligned = false;

    float BEARING_TOLERANCE = 2.0;

    while (!isAligned) {
        updateMPU();
        float currentRelative = getCurrentAbsoluteBearing();
        float error = targetBearing - currentRelative;

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
            maintainBearing();
        }

        Serial.print(F("Aligning to bearing: "));
        Serial.print(currentRelative);

        delay(5);
    }

    stopMotors();
    Serial.println(F("Alignment complete"));
}

//turns right 90 degrees
void turn_right_90() {
    float targetBearing = currentBearing - 90;
    if (targetBearing < 0) targetBearing += 360;

    Serial.print(F("Turning right 90 to "));
    Serial.println(targetBearing);

    analogWrite(FNA, 150);
    analogWrite(FNB, 150);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    float startYaw = yaw;
    float targetYaw = startYaw - 80.0;

    while (yaw > targetYaw) {
        updateMPU();

        delay(5);
    }

    stopMotors();
    Serial.println(F("Rough turn complete"));

    currentBearing = targetBearing;
    alignToBearing(targetBearing);

    unsigned long alignmentStartTime = millis();
    bool isAligned = false;

    float BEARING_TOLERANCE = 2.0;

    while (!isAligned) {
        updateMPU();
        float currentRelative = getCurrentAbsoluteBearing();
        float error = targetBearing - currentRelative;

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
            maintainBearing();
        }

        delay(5);
    }

    stopMotors();
    Serial.println(F("Alignment complete"));
}

//turns 180 according to bearing
void turn_180() {
    float targetBearing = currentBearing + 180;
    if (targetBearing >= 360) targetBearing -= 360;

    Serial.print(F("Turning 180 to "));
    Serial.println(targetBearing);

    analogWrite(FNA, 120);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 120);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    float startYaw = yaw;
    float targetYaw = startYaw + 170;

    while (startYaw < targetYaw) {
        updateMPU();

        delay(5);
    }

    stopMotors();

    currentBearing = targetBearing;
    alignToBearing(targetBearing);

    unsigned long alignmentStartTime = millis();
    bool isAligned = false;
    float BEARING_TOLERANCE = 2.0;

    while (!isAligned) {
        updateMPU();
        float currentRelative = getCurrentAbsoluteBearing();
        float error = targetBearing - currentRelative;

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
            maintainBearing();
        }

        delay(5);
    }

    stopMotors();
    Serial.println(F("Alignment complete"));
}

// EEPROM Memory Functions
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
    uint8_t addr = 0;
    uint8_t check = 0;

    if (EEPROM.get(addr, check) == 0xFFFF) 
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
    turn_180();
    delay(5000);

    for (count -= 1; count > junction_nodes[index]; count--)
    {
        Serial.print("Count: ");
        Serial.println(count);

        if (movement_arr[count] == 'F')
        {
            Serial.println("Move forward now");
            moveForward(100);
        }
        else if (movement_arr[count] == 'L')
        {
            Serial.println("Turn right now");
            turn_right_90();
        }
        else if (movement_arr[count] == 'R')
        {
            Serial.println("Turn left now");
            turn_left_90();
        }
    }

    // offset comepensation for front moevement from junction node only
    if (movement_arr[junction_nodes[index]] == 'F')
    {
        Serial.println("Move forward now");
        moveForward(100);
    }

    // Reorientation
    if (movement_arr[count] == 'F')
    {
        Serial.println("Reorienting: F (Forward)");
        Serial.println("Turn 180 now");
        turn_180();
        junction_visited[index] = 1;
        Serial.println("Junction visited stored as 1.");
    }
    else if (movement_arr[count] == 'L')
    {
        Serial.println("Reorienting: L (Left)");
        Serial.println("Turn left now");
        turn_left_90();
        junction_visited[index] = 2;
        Serial.println("Junction visited stored as 2.");
    }
    else if (movement_arr[count] == 'R')
    {
        Serial.println("Reorienting: R (Right)");
        Serial.println("Turn right now");
        turn_right_90();
        junction_visited[index] = 3;
        Serial.println("Junction visited stored as 3.");
    }
}

void search_maze()
{
    Serial.println("Searching maze.");
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

    Serial.println("-----");
    Serial.print("Index: ");
    Serial.println(index);
    Serial.print("Junction visited: ");
    Serial.println(junction_visited[index]);
    Serial.println("-----");

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
        Serial.println("Front space detected");
        if (left == 1 || right == 1)
        {
            Serial.println("Left and/or right space detected");
            index++;
            junction_nodes[index] = count;
            Serial.println("Junction node stored.");
        }

        Serial.println("Move forward now.");
        moveForward(100);
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else if (left != 0 && (count != junction_nodes[index] || junction_visited[index] < 2)) // left has space
    {
        Serial.println("Left space detected");
        if (right == 1)
        {
            Serial.println("Right space detected");
            index++;
            junction_nodes[index] = count;
            Serial.println("Junction node stored.");
        }

        Serial.println("Turn left now");
        turn_left_90();
        Serial.println("Turn left 90° done.");
        movement_arr[count] = 'L';
        count++;
        Serial.println("Left turn stored.");

        Serial.println("Move forward now.");
        moveForward(100);
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else if (right != 0 && (count != junction_nodes[index] || junction_visited[index] < 3)) // right has space
    {
        Serial.println("Right space detected");

        Serial.println("Turn right now");
        turn_right_90();
        Serial.println("Turn right 90° done.");
        movement_arr[count] = 'R';
        count++;
        Serial.println("Right turn stored.");

        Serial.println("Move forward now.");
        moveForward(100);
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
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
        Serial.println("backtrack_and_reorienting complete.");
    }
}

void init_arrays()
{
    Serial.println("Initializing arrays.");

    memset(movement_arr, 0, sizeof(movement_arr));
    memset(junction_nodes, 0, sizeof(junction_nodes));
    memset(junction_visited, 0, sizeof(junction_visited));

    // memoryReset();
    // Serial.println("EEPROM reseted.");
}

void setup()
{
    Serial.begin(9600);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(FNA, OUTPUT);
    pinMode(FNB, OUTPUT);
    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
    attachPCINT(digitalPinToPCINT(LEFT_ENCODER_PIN), leftEncoderISP, CHANGE);
    attachPCINT(digitalPinToPCINT(RIGHT_ENCODER_PIN), rightEncoderISP, CHANGE);
    ultrasonicSetup(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    ultrasonicSetup(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    ultrasonicSetup(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    calculateError();

    delay(1000);
    updateMPU();

    initialYaw = yaw;

    Serial.print(F("Initial yaw set to: "));
    Serial.print(initialYaw);
    Serial.println("Setup complete.");

    init_arrays();
}

void loop()
{
    if (flags.is_LeBron_done == 0) 
    {
        search_maze();
    }
    else
    {
        if (flags.has_LeBron_written == 0)
        {
            memoryWrite();
            flags.has_LeBron_written = 1;
        }
    }
}