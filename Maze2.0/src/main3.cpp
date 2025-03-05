#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <EEPROM.h>
#include <array>
#include <cstring>

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
#define RIGHT_MOTOR_CALIBRATION 0.95  // Reduce right motor speed if it's stronger

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
int currentBearing = NORTH;  // Start with North as default bearing
bool correctionEnabled = true;

// Correction timing variables
unsigned long lastCorrectionTime = 0;
const unsigned long CORRECTION_INTERVAL = 500; // milliseconds

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

// shortform gooning
using goon_char = std::array<char, MAX_GOONS>;
using goon_int = std::array<int, MAX_GOONS>;

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

inline int checkTheDih (int trigPin, int echoPin) // Treshold distance for junction 
{
    return getDistance( trigPin, echoPin) <= 10 ? 0 : 1; 
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
float initialYaw = 0.0;  // Store the initial yaw to use as reference
const float ALPHA = 0.96; // Complementary filter coefficient

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
    // More samples for better calibration
    for (int i = 0; i < 1000; i++) {
        getOrientation();
        GyroErrorX += gyroX;
        GyroErrorY += gyroY;
        GyroErrorZ += gyroZ;
        delay(1); // Small delay for more stable sampling
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
    
    // Correct gyro measurements
    gyroX -= GyroErrorX;
    gyroY -= GyroErrorY;
    gyroZ -= GyroErrorZ;
    
    // Calculate angles from accelerometer data
    accelAngleX = atan2(accelY, sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180 / PI;
    accelAngleY = atan2(-accelX, sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180 / PI;
    
    // Complementary filter for roll and pitch
    roll = ALPHA * (roll + gyroX * elapsedTime) + (1 - ALPHA) * accelAngleX;
    pitch = ALPHA * (pitch + gyroY * elapsedTime) + (1 - ALPHA) * accelAngleY;
    
    // Only gyro for yaw (no magnetometer)
    yaw += gyroZ * elapsedTime;
}

// Function to normalize yaw to 0-360 range
float normalizeYaw(float rawYaw) {
    float normalized = fmod(rawYaw, 360.0);
    if (normalized < 0) normalized += 360.0;
    return normalized;
}

// Function to calculate the current absolute bearing based on initial reference
int getCurrentAbsoluteBearing() {
    // Calculate relative yaw from the initial position
    float relativeYaw = yaw - initialYaw;
    
    // Normalize to 0-360 range
    float normalizedYaw = normalizeYaw(relativeYaw);
    
    // Round to the nearest 90-degree bearing
    int absoluteBearing = round(normalizedYaw / 90) * 90;
    if (absoluteBearing >= 360) absoluteBearing = 0;
    
    return absoluteBearing;
}

// Function to turn right by 90 degrees (adjusts bearing)
void turnRight90() {
    // Calculate new bearing after a right turn
    int newBearing = currentBearing - 90;
    if (newBearing < 0) newBearing += 360;
    
    Serial.print("Turning right 90° from bearing ");
    Serial.print(currentBearing);
    Serial.print("° to ");
    Serial.print(newBearing);
    Serial.println("°");
    
    // Start turning right
    analogWrite(FNA, 100 * LEFT_MOTOR_CALIBRATION);  // Left motor forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, 100 * RIGHT_MOTOR_CALIBRATION);  // Right motor backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    
    // Turn roughly 80 degrees (allows for overshoot)
    float startYaw = yaw;
    float targetYaw = startYaw - 80.0;
    
    while (yaw > targetYaw) {
        updateMPU();
        
        // Print current yaw for debugging
        Serial.print("Turning: Current Yaw = ");
        Serial.print(yaw);
        Serial.print(", Target = ");
        Serial.println(targetYaw);
        
        delay(10);  // Small delay for stability
    }
    
    // Stop motors after rough turn
    stopMotors();
    Serial.println("Rough turn complete");
    
    // Update current bearing
    currentBearing = newBearing;
    
    // Fine-tune alignment with the new bearing
    alignToBearing(newBearing);
}

// Function to turn left by 90 degrees (adjusts bearing)
void turnLeft90() {
    // Calculate new bearing after a left turn
    int newBearing = currentBearing + 90;
    if (newBearing >= 360) newBearing -= 360;
    
    Serial.print("Turning left 90° from bearing ");
    Serial.print(currentBearing);
    Serial.print("° to ");
    Serial.print(newBearing);
    Serial.println("°");
    
    // Start turning left
    analogWrite(FNA, 130 * LEFT_MOTOR_CALIBRATION);  // Left motor backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 130 * RIGHT_MOTOR_CALIBRATION);  // Right motor forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    // Turn roughly 80 degrees (allows for overshoot)
    float startYaw = yaw;
    float targetYaw = startYaw + 80.0;
    
    while (yaw < targetYaw) {
        updateMPU();
        
        // Print current yaw for debugging
        Serial.print("Turning: Current Yaw = ");
        Serial.print(yaw);
        Serial.print(", Target = ");
        Serial.println(targetYaw);
        
        delay(10);  // Small delay for stability
    }
    
    // Stop motors after rough turn
    stopMotors();
    Serial.println("Rough turn complete");
    
    // Update current bearing
    currentBearing = newBearing;
    
    // Fine-tune alignment with the new bearing
    alignToBearing(newBearing);
}

// Function to turn 180 degrees (spin to the opposite bearing)
void turn180() {
    // Calculate the new bearing after a 180-degree turn
    int newBearing = currentBearing + 180;
    if (newBearing >= 360) newBearing -= 360;

    Serial.print("Turning 180° from bearing ");
    Serial.print(currentBearing);
    Serial.print("° to ");
    Serial.print(newBearing);
    Serial.println("°");

    // Start turning (choose either left or right turn)
    // Here, we'll use a left turn for 180 degrees
    analogWrite(FNA, 100 * LEFT_MOTOR_CALIBRATION);  // Left motor backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 100 * RIGHT_MOTOR_CALIBRATION);  // Right motor forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    // Turn roughly 170 degrees (allows for overshoot)
    float startYaw = yaw;
    float targetYaw = startYaw + 170.0;  // Adjust for overshoot

    while (yaw < targetYaw) {
        updateMPU();

        // Print current yaw for debugging
        Serial.print("Turning: Current Yaw = ");
        Serial.print(yaw);
        Serial.print(", Target = ");
        Serial.println(targetYaw);

        delay(10);  // Small delay for stability
    }

    // Stop motors after rough turn
    stopMotors();
    Serial.println("Rough 180° turn complete");

    // Update current bearing
    currentBearing = newBearing;

    // Fine-tune alignment with the new bearing
    alignToBearing(newBearing);
}

// Function to align with the nearest cardinal direction
void alignToBearing(int targetBearing) {
    // Get the current absolute bearing
    int currentAbsoluteBearing = getCurrentAbsoluteBearing();
    float currentRelativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(currentRelativeYaw);
    
    // Calculate the error (shortest path to target bearing)
    float error = targetBearing - normalizedYaw;
    
    // Adjust for shortest turn direction (never turn more than 180 degrees)
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    Serial.print("Aligning to bearing: ");
    Serial.print(targetBearing);
    Serial.print("°, Current normalized yaw: ");
    Serial.print(normalizedYaw);
    Serial.print("°, Error: ");
    Serial.println(error);
    
    // Define tolerance for alignment
    const float BEARING_TOLERANCE = 2.0;
    
    // Only correct if error is outside tolerance
    if (abs(error) > BEARING_TOLERANCE) {
        // Determine turn direction
        if (error > 0) {
            // Turn Left (CCW)
            analogWrite(FNA, 80 * LEFT_MOTOR_CALIBRATION);  // Left motor backward
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            analogWrite(FNB, 80 * RIGHT_MOTOR_CALIBRATION);  // Right motor forward
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            Serial.println("Turning LEFT to align");
        } else {
            // Turn Right (CW)
            analogWrite(FNA, 80 * LEFT_MOTOR_CALIBRATION);  // Left motor forward
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            analogWrite(FNB, 80 * RIGHT_MOTOR_CALIBRATION);  // Right motor backward
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            Serial.println("Turning RIGHT to align");
        }
        
        // Continue turning until aligned (with tighter tolerance for stopping)
        float currentError;
        do {
            updateMPU();
            
            // Recalculate the error
            currentRelativeYaw = yaw - initialYaw;
            normalizedYaw = normalizeYaw(currentRelativeYaw);
            currentError = targetBearing - normalizedYaw;
            
            // Adjust for shortest turn direction
            if (currentError > 180) currentError -= 360;
            if (currentError < -180) currentError += 360;
            
            Serial.print("Aligning - Current yaw: ");
            Serial.print(normalizedYaw);
            Serial.print("°, Error: ");
            Serial.println(currentError);
            
            delay(10);  // Small delay for stability
        } while (abs(currentError) > BEARING_TOLERANCE * 0.5);  // Tighter tolerance for stopping
        
        // Stop motors after alignment
        stopMotors();
        Serial.println("Alignment complete");
        
        // Update current bearing
        currentBearing = targetBearing;
    }
}

// Improved function to check and correct bearing with proportional control
void maintainBearing() {
    if (!correctionEnabled) return;
    
    // Check if we need to align to the current bearing
    float currentRelativeYaw = yaw - initialYaw;
    float normalizedYaw = normalizeYaw(currentRelativeYaw);
    
    // Calculate error from current bearing
    float error = currentBearing - normalizedYaw;
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    // Define tolerance for alignment
    const float BEARING_TOLERANCE = 3.0;
    
    // Only correct if error is outside tolerance
    if (abs(error) > BEARING_TOLERANCE) {
        Serial.print("Bearing correction needed. Error: ");
        Serial.println(error);
        
        // Calculate proportional correction values (start with small values)
        int correctionPWM = min(abs(error) * 2, 50); // Limit maximum correction
        
        if (isMovingForward) {
            if (error > 0) {
                // Need to turn slightly left
                analogWrite(FNA, (100 * LEFT_MOTOR_CALIBRATION) - correctionPWM);
                analogWrite(FNB, (100 * RIGHT_MOTOR_CALIBRATION) + correctionPWM);
            } else {
                // Need to turn slightly right
                analogWrite(FNA, (100 * LEFT_MOTOR_CALIBRATION) + correctionPWM);
                analogWrite(FNB, (100 * RIGHT_MOTOR_CALIBRATION) - correctionPWM);
            }
            
            // Brief correction pulse
            delay(50);
            
            // Resume normal speed
            MoveForward(100); // Or whatever your normal PWM value is
        } else {
            // If not moving, do a stationary correction
            alignToBearing(currentBearing);
        }
    }
}

void MoveForward(int PWM) {
    isMovingForward = true;  // Fixed: Was using undefined variable 'forward'
    analogWrite(FNA, PWM);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, PWM);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving Forward...");  // Added feedback message
}

void moveForwards(int PWM) {
    if (!isMovingForward && !targetReached) {
        // Reset distance counters before movement starts
        leftTotalDistance = 0.0;
        rightTotalDistance = 0.0;
        targetReached = false;
    }

    // Start moving forward
    isMovingForward = true;
    analogWrite(FNA, PWM * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, PWM * RIGHT_MOTOR_CALIBRATION);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    while (isMovingForward && !targetReached) {
        // Update MPU6050 data for bearing correction
        updateMPU();

        // Calculate the correction needed based on yaw
        float correction = initialYaw - yaw; // Turn right is negative

        // Apply proportional correction to motors
        if (correction > 3) { // Need to turn left
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else if (correction < -3) { // Need to turn right
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        } else { // No correction needed
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        }

        // Update distance traveled
        updateDistance();

        // Calculate average distance
        float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;

        // Stop if the target distance is reached
        if (avgDistance >= TARGET_DISTANCE && !targetReached) {
            stopMotors();
            targetReached = true;
            break; // Exit the loop
        }

        delay(5); // Small delay for stability
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

//-----------------------ALGORITM---------------------------------------------------------
// who tf spelt algorithm liddat
int back_it_up_bih (int back_index, goon_char temp_movement, goon_int movement_index, int* index_counter)
{
    turn180();            // turn away from gyat damn wall

    // **********************************************************************
    // test: trial and error weather i > OR i >= should be used 
    // **********************************************************************
    for (int i = back_index; i > movement_index[index_counter]; i--)
    {
        if (temp_movement[i] = 'F')
        {
            moveForwards(100);
        }
        else if (temp_movement[i] = 'L')
        {
           turnRight90(); // function to TURN RIGHT *** OPPOSITE DIRECTION
        }
        else if (temp_movement[i] = 'R')
        {
            turnLeft90(); // function to TURN LEFT *** OPPOSITE DIRECTION
        }
    }

    // this bih has returned to most recent junction
    return movement_index[index_counter];
}

void keep_going_daddy (goon_char temp_movement, goon_int movement_index, goon_int junction_gooned, int* index_counter, int* i)
{
    int front, left, right;    

    for (i = i; i < MAX_GOONS; i++)
    {
        front = checkTheDih(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
        left = checkTheDih(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
        right = checkTheDih(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

        if (junction_gooned[index_counter] < 1 && front)          // if there is a baddie in front && the baddie has not been visited before
        {
            // save at which movement/node there is a junction
            if (left || right) movement_index[index_counter++] = i;     
            moveForwards(255);
            temp_movement[i] = 'F';
        }
        else if (junction_gooned[index_counter] < 2 && left)      // if there is a baddie to the left && the baddie has not been visited before
        {
            // save at which movement/node there is a junction
            if (right) movement_index[index_counter++] = i;     
            turnLeft90();
            temp_movement[i] = 'L';
            moveForwards(255);
            temp_movement[i++] = 'F';
        }
        else if (junction_gooned[index_counter] < 3 && right)     // if there is a baddie to the right && the baddie has not been visited before
        {
            // save at which movement/node there is a junction
            // if (left) movement_index[index_counter++] = i;     
            turnRight90();
            temp_movement[i] = 'R';
            moveForwards(255);
            temp_movement[i++] = 'F';
        }
        else                // there is no baddie in front, left or right, i.e. a dead end
        {
            // **********************************************************************
            // remember to update/decrement the index_counter
            // along with reset the junction_gooned flag
            // **********************************************************************
            i = back_it_up_bih (i, temp_movement, movement_index, &index_counter);

            // check which direction did this bih last take to go into that junction & 
            // set prev_junction flag to indicate which direction this bih should avoid
            // for junction_gooned:
            // if last movement was forwards = set 1 ,    if last movement was left = set 2,   if right = set 3
            // index_counter++ is not needed as we can just access the same array position as movement_index for simplicity
            // basically the node movement_index is referring to, will have a corresponding value in junction_gooned

            // **********************************************************************
            // check whether i or ++i should be used
            // **********************************************************************
            if (temp_movement[i + 1] = 'F')
            {
                turn180();                              // reorient the bih
                junction_gooned[index_counter] = 1;     // flag the junction has been gooned
            }
            else if (temp_movement[i + 1] = 'L')
            {
                turnLeft90();                           // reorient the bih
                junction_gooned[index_counter] = 2;     // flag the junction has been gooned
            }
            else if (temp_movement[i + 1] = 'R')
            {
                turnRight90();                          // reorient the bih
                junction_gooned[index_counter] = 3;     // flag the junction has been gooned
            }
        }
    }
    return temp_movement;
}

int follow_gooning_path(goon_char temp_movement, goon_int movement_index, int* index_counter)
{
    // follow the path that was stored in the memory up until last junction
    int i;
    for (i = 0; i < movement_index[index_counter] + 1; i++)
    {
        if (temp_movement[i] = 'F')
        {
            moveForwards(100);
        }
        else if (temp_movement[i] = 'L')
        {
            turnLeft90();
            moveForwards(100);
        }
        else if (temp_movement[i] = 'R')
        {
            turnRight90();
            moveForwards(100);
        }
    }
    return i + 1;               // i + 1 so keep_going_daddy can continue from the next array index

    /*
    int front, left, right;
    front = checkTheDih(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    left = checkTheDih(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    right = checkTheDih(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

    // check what was the next movement in the previous path
    if (temp_movement[movement_index[index_counter]] = 'F')
    {
        
    }
    else if (temp_movement[movement_index[index_counter]] = 'L')
    {
        
    }
    else if (temp_movement[movement_index[index_counter]] = 'R')
    {
        
    }
    */
}


// Reset memory in EEPROM
void memoryReset()
{
    for(int i = 0; i < EEPROM.length(); i++)
    {
        EEPROM.write(i, 0);
    }
}

// Write memory to EEPROM
void memoryWrite(char* input)
{
    for(int i = 0; i < MAX_GOONS; i++) 
    {
        EEPROM.write(i, input[i]);
    }
}

// Read and Return all value from EEPROM
void memoryRead(goon_char& buffer)
{
    //char* buffer = "";
    // i = 0; i < temp_movement.size();
    // i = temp_movement.size() + 1; i < movement_index.size();
    int index = 0;
    for(int i = 0; i < EEPROM.length(); i++) 
    {
        char temp = EEPROM.read(i);
        if(temp != 0)
        {
            buffer[i] = temp;
            index++;
        }
        else
        {
            break;
        }
    }
    buffer[index] = '\0';   // add null terminator to the end of the string
}

void start_gooning ()
{
    // initialize arrays with std::array
    // gooning version
    goon_char temp_movement;
    goon_char final_movement;
    
    // these 2 will utilise the same index, i.e. index_counter
    goon_int movement_index;
    goon_int junction_gooned;

    // Initialize the arrays if needed
    memset(temp_movement.data(), 0, temp_movement.size());
    memset(final_movement.data(), 0, final_movement.size());
    memset(movement_index.data(), 0, movement_index.size());
    memset(junction_gooned.data(), 0, junction_gooned.size());

    int index_counter = 0;
    int dih = 0;

    memoryRead(final_movement);

    if (final_movement[0] == '\0')                    // if no gooning pattern found in memory then start a new goon pattern
    {
        keep_going_daddy(temp_movement, movement_index, junction_gooned, &index_counter, &dih);
    }
    else
    {
        memcpy(final_movement.data(), temp_movement.data(), final_movement.size());
        dih = follow_gooning_path(final_movement, movement_index, &index_counter);
        keep_going_daddy(temp_movement, movement_index, junction_gooned, &index_counter, &dih);
    }

    if (strlen(temp_movement) < strlen(final_movement))
    {
        // reset the memory so can save new goon pattern 
        memoryReset();
        memoryWrite(temp_movement);
        // **********************************************************************
        // add movement_index, index_counter to memory
        // **********************************************************************
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
    Wire.beginTransmission(MPU);  // Added: Initialize MPU communication
    Wire.write(0x6B);             // PWR_MGMT_1 register
    Wire.write(0);                // Wake up the MPU-6050
    Wire.endTransmission(true);
    calculateError();
    
    // Reset distance counters
    leftTotalDistance = 0.0;
    rightTotalDistance = 0.0;
    targetReached = false;

    // Let the MPU stabilize and then store the initial yaw as reference
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

//=============================NEW 90 DEGREE==========================================================
  updateMPU();
  if (!done) {

    float normalizedYaw = normalizeYaw(yaw - initialYaw);

    turnLeft90();

    maintainBearing();

    done = true;
  }
  
  alignToBearing(currentBearing);
  Serial.println(currentBearing);
  

//==========================ULTRASONIC SENSOR=========================================================
    /*// Read distances from all three ultrasonic sensors
    float frontDistance = getDistance(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    float leftDistance = getDistance(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    float rightDistance = getDistance(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

    // Display ultrasonic sensor readings
    Serial.print("Front: ");
    Serial.print(frontDistance);
    Serial.print(" cm | Left: ");
    Serial.print(leftDistance);
    Serial.print(" cm | Right: ");
    Serial.print(rightDistance);
    Serial.println(" cm");*/

//===========================Ultrasonic 25cm====================================================
// if (!hasStarted) {
//     hasStarted = true;
//     initialDistance = getDistance(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
//   } else {
//     float currentDistance = getDistance(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
//     float deltaDistance = initialDistance - currentDistance;
//     initialDistance = currentDistance;
//     totalDistance += deltaDistance;
//   }
//   if (totalDistance > 18) { //Stops at 25cm, 7cm(overshoot)  offset 
//     Serial.println("Done");
//     stopMotors();
//   } else {
//     MoveForward(100);
//  delay(5);
//   }


//===========================MOTOR CONTROL + ROTARY ENCODER===========================================
    // //Move forward for 25 cm
    // if (!targetReached) {  // Added check to prevent continuous movement after target is reached
    //     MoveForward(255);
    // }

    // // Update and display distance traveled by the wheels
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
    //     stopMotors();
    //     targetReached = true;
    // }

//===============================25cm===================================================================
    /*if (!targetReached) {
        moveForwards(100); // Move forward with PWM = 100
    } else {
        Serial.println("Target distance reached.");
    }*/


    // if (!isMovingForward && !targetReached) {
    //     // Reset distance counters before movement starts
    //     leftTotalDistance = 0.0;
    //     rightTotalDistance = 0.0;
    //     targetReached = false;

    //     // Start moving
    //     moveForwards(100);
    // }

    // // Update distance traveled
    // updateDistance();

    // // Check if the car has reached exactly 25 cm
    // float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;
    // if (avgDistance >= 25.0 && !targetReached) {
    //     stopMotors();
    //     targetReached = true;
    // }

    // // Print distance values for debugging
    // Serial.print("Left Distance: ");
    // Serial.print(leftTotalDistance);
    // Serial.print(" cm | Right Distance: ");
    // Serial.print(rightTotalDistance);
    // Serial.print(" cm | Avg Distance: ");
    // Serial.print(avgDistance);
    // Serial.println(" cm");

    // delay(10);  // Small delay to ensure smooth execution

//===============================MPU-6050==============================================================

    //     // Update MPU6050 angles
    //     updateMPU();

    //     // Print MPU6050 angles
    //     Serial.print("Roll: ");
    //     Serial.print(roll);
    //     Serial.print("° | Pitch: ");
    //     Serial.print(pitch);
    //     Serial.print("° | Yaw: "); //(Turning angle) negative for right turn and positive for left hand turn 
    //     Serial.print(yaw);
    //     Serial.println("°");

    //    delay(100);  // Added delay for stability

//=================================90 RIGHT & LEFT============================================================
    // // Update MPU6050 angles
    // updateMPU();
    
    // // Print current bearing information
    // float relativeYaw = yaw - initialYaw;
    // float normalizedYaw = normalizeYaw(relativeYaw);
    
    // Serial.print("Current Yaw: ");
    // Serial.print(yaw);
    // Serial.print("°, Normalized Yaw: ");
    // Serial.print(normalizedYaw);
    // Serial.print("°, Current Bearing: ");
    // Serial.print(currentBearing);
    // Serial.println("°");
    
    // // Static variable to ensure the turn only happens once
    // static bool turnCompleted = false;
    
    // // Execute the turn only once
    // if (!turnCompleted) {
    //     turnRight90();  // Or turnLeft90() depending on what you want to test
    //     turnCompleted = true;
    //     Serial.println("Turn and stop sequence completed");
    // }
    
    // // Continuously check and correct bearing, but not too frequently
    // if (millis() - lastCorrectionTime > CORRECTION_INTERVAL) {
    //     maintainBearing();
    //     lastCorrectionTime = millis();
    // }
    
    // // Keep updating distance if moving forward
    // if (isMovingForward) {
    //     updateDistance();
    // }
    
    // delay(100);  // Small delay for more responsive bearing corrections

//================================180 TURN======================================================================
    // Update MPU6050 angles
    // updateMPU();
    
    // // Print current bearing information
    // float relativeYaw = yaw - initialYaw;
    // float normalizedYaw = normalizeYaw(relativeYaw);
    
    // Serial.print("Current Yaw: ");
    // Serial.print(yaw);
    // Serial.print("°, Normalized Yaw: ");
    // Serial.print(normalizedYaw);
    // Serial.print("°, Current Bearing: ");
    // Serial.print(currentBearing);
    // Serial.println("°");
    
    // // Static variable to ensure the turn only happens once
    // static bool turnCompleted = false;
    
    // // Execute the turn only once
    // if (!turnCompleted) {
    //     turn180();  // Or turnLeft90() depending on what you want to test
    //     turnCompleted = true;
    //     Serial.println("Turn and stop sequence completed");
    // }
    
    // // Continuously check and correct bearing, but not too frequently
    // if (millis() - lastCorrectionTime > CORRECTION_INTERVAL) {
    //     maintainBearing();
    //     lastCorrectionTime = millis();
    // }
    
    // // Keep updating distance if moving forward
    // if (isMovingForward) {
    //     updateDistance();
    // }
    
    // delay(100);  // Small delay for more responsive bearing corrections
    //}

}