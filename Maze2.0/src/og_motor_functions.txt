#include "og_motor_functions_header.h"

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

int currentBearing = NORTH;  // Start with North as default bearing
bool correctionEnabled = true;

// Correction timing variables
unsigned long lastCorrectionTime = 0;
const unsigned long CORRECTION_INTERVAL = 500; // milliseconds

bool hasStarted = false;
float initialDistance = 0;
float totalDistance = 0;

// RPM calculation variables
unsigned long lastLeftPulseTime = 0;
unsigned long lastRightPulseTime = 0;
float leftRPM = 0.0;
float rightRPM = 0.0;

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

BooleanFlags flags = {0, 0, 1, 0, 0};

// Interrupt service routines
void leftEncoderISR() { leftPulses++; }

void rightEncoderISR() { rightPulses++; }

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

int checkDist(int trigPin, int echoPin) {
    float distance = getDistance(trigPin, echoPin);
      if (distance > 75) return -1; 
      else if (distance <= 10) return 0;
      else return 1;
}


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
void turn_right_90() {
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
void turn_left_90() {
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
void turn_180() {
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

void Forward25(int PWM) {
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