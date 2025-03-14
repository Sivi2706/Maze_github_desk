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
const unsigned int PULSES_PER_TURN = 20;
const float WHEEL_DIAMETER = 4;
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

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

EncoderState encoderState;
MotorState motorState;
MPUState mpuState;
BearingState bearingState;

// Interrupt service routines
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
                analogWrite(FNA, (110 * LEFT_MOTOR_CALIBRATION) - correctionPWM);
                analogWrite(FNB, (110 * RIGHT_MOTOR_CALIBRATION) + correctionPWM);
            } else {
                analogWrite(FNA, (110 * LEFT_MOTOR_CALIBRATION) + correctionPWM);
                analogWrite(FNB, (110 * RIGHT_MOTOR_CALIBRATION) - correctionPWM);
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

// void turnLeft90(MPUState &mpu, BearingState &bearing) {
//     float newRelativeBearing = bearing.currentRelativeBearing + 90;
//     if (newRelativeBearing >= 360) newRelativeBearing -= 360;
    
//     Serial.print("Turning left 90° from relative bearing ");
//     Serial.print(bearing.currentRelativeBearing);
//     Serial.print("° to ");
//     Serial.print(newRelativeBearing);
//     Serial.println("°");
    
//     // digitalWrite(IN1, LOW);
//     // digitalWrite(IN2, HIGH);
//     // digitalWrite(IN3, HIGH);
//     // digitalWrite(IN4, LOW);
//     // analogWrite(FNA, 255 * LEFT_MOTOR_CALIBRATION);
//     // analogWrite(FNB, 255 * RIGHT_MOTOR_CALIBRATION);
//     // delay(15);
//     // analogWrite(FNA, 100 * LEFT_MOTOR_CALIBRATION);
//     // analogWrite(FNB, 100 * RIGHT_MOTOR_CALIBRATION);
    
//     float startYaw = mpu.yaw;
//     float targetYaw = startYaw + 90.0;
    
//     while (mpu.yaw < targetYaw) {
//         updateMPU(mpu);
//         Serial.print("Turning: Current Yaw = ");
//         Serial.print(mpu.yaw);
//         Serial.print(", Target = ");
//         Serial.println(targetYaw);
//         delay(1);
//     }
    
//     stopMotors();
//     Serial.println("Rough turn complete");
    
//     bearing.currentRelativeBearing = newRelativeBearing;
//     alignToBearing(mpu, bearing, newRelativeBearing);
    
//     unsigned long alignmentStartTime = millis();
//     bool isAligned = false;
    
//     while (!isAligned) {
//         updateMPU(mpu);
//         float currentRelative = getCurrentRelativeBearing(mpu, bearing);
//         float error = newRelativeBearing - currentRelative;
        
//         if (error > 180) error -= 360;
//         if (error < -180) error += 360;
        
//         if (abs(error) <= BEARING_TOLERANCE) {
//             if (alignmentStartTime == 0) {
//                 alignmentStartTime = millis();
//             }
            
//             if (millis() - alignmentStartTime >= 500) {
//                 isAligned = true;
//             }
//         } else {
//             alignmentStartTime = 0;
//             maintainBearing(mpu, bearing, motorState);
//         }
        
//         Serial.print("Aligning - Current relative bearing: ");
//         Serial.print(currentRelative);
//         Serial.print("°, Error: ");
//         Serial.println(error);
        
//         delay(1);
//     }
    
//     stopMotors();
//     Serial.println("Alignment complete. Robot is at target bearing.");
// }
// void turnLeft90(MPUState &mpu, BearingState &bearing) {
//   float newRelativeBearing = bearing.currentRelativeBearing + 90;
//   if (newRelativeBearing > 360) newRelativeBearing -= 360;

//   Serial.print(F("Turning left 90 from relative bearing "));
//   Serial.print(bearing.currentRelativeBearing);
//   Serial.print(" to ");
//   Serial.println(newRelativeBearing);

//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, HIGH);
//   digitalWrite(IN3, HIGH);
//   digitalWrite(IN4, LOW);
//   analogWrite(FNA, 255 * LEFT_MOTOR_CALIBRATION);
//   analogWrite(FNB, 255 * RIGHT_MOTOR_CALIBRATION);
//   delay(15);
//   analogWrite(FNA, 200 * LEFT_MOTOR_CALIBRATION);
//   analogWrite(FNB, 200 * RIGHT_MOTOR_CALIBRATION);

//   float startYaw = mpu.yaw;
//   float targetYaw = startYaw - 90.0;
  
//   while (mpu.yaw < targetYaw) {
//       updateMPU(mpu);
//       Serial.print("Turning: Current Yaw = ");
//       Serial.print(mpu.yaw);
//       Serial.print(", Target = ");
//       Serial.println(targetYaw);
//       delay(1);
//   }

//   stopMotors();
//   Serial.println("Rough turn complete");
  
//   bearing.currentRelativeBearing = newRelativeBearing;
//   alignToBearing(mpu, bearing, newRelativeBearing);
  
//   unsigned long alignmentStartTime = millis();
//   bool isAligned = false;
  
//   while (!isAligned) {
//       updateMPU(mpu);
//       float currentRelative = getCurrentRelativeBearing(mpu, bearing);
//       float error = newRelativeBearing - currentRelative;
      
//       if (error > 180) error -= 360;
//       if (error < -180) error += 360;
      
//       if (abs(error) <= BEARING_TOLERANCE) {
//           if (alignmentStartTime == 0) {
//               alignmentStartTime = millis();
//           }
          
//           if (millis() - alignmentStartTime >= 500) {
//               isAligned = true;
//           }
//       } else {
//           alignmentStartTime = 0;
//           maintainBearing(mpu, bearing, motorState);
//       }
      
//       Serial.print("Aligning - Current relative bearing: ");
//       Serial.print(currentRelative);
//       Serial.print("°, Error: ");
//       Serial.println(error);
      
//       delay(1);
//   }
  
//   stopMotors();
//   Serial.println("Alignment complete. Robot is at target bearing.");
// }

//================================
void turnLeft90(MPUState &mpu, BearingState &bearing) {
    float newRelativeBearing = bearing.currentRelativeBearing + 90;
    if (newRelativeBearing > 360) newRelativeBearing -= 360;
    
    Serial.print("Turning left 90° from relative bearing ");
    Serial.print(bearing.currentRelativeBearing);
    Serial.print("° to ");
    Serial.print(newRelativeBearing);
    Serial.println("°");

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(FNA, 255 * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, 255 * RIGHT_MOTOR_CALIBRATION);
    delay(10);
    analogWrite(FNA, 110 * LEFT_MOTOR_CALIBRATION);  // Adjusted to match turnRight90
    analogWrite(FNB, 110 * RIGHT_MOTOR_CALIBRATION); // Adjusted to match turnRight90
    
    float startYaw = mpu.yaw;
    float targetYaw = startYaw + 90.0;  // Corrected: Left turn increases yaw
    
    while (mpu.yaw < targetYaw) {  // Corrected: Left turn increases yaw, so < targetYaw
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

//================================

void turnRight90(MPUState &mpu, BearingState &bearing) {
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
    digitalWrite(IN4, LOW);
    analogWrite(FNA, 255 * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, 255 * RIGHT_MOTOR_CALIBRATION);
    delay(10);
    analogWrite(FNA, 110 * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, 110 * RIGHT_MOTOR_CALIBRATION);
    
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

void turn180(MPUState &mpu, BearingState &bearing) {
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
    Serial.println("Starting with relative bearing of 0 degrees");
}

void loop() {
    static unsigned long lastPrintTime = 0;
    const unsigned long printInterval = 500;

    updateMPU(mpuState);

    if (millis() - lastPrintTime >= printInterval) {
        printCurrentBearing(mpuState, bearingState);
        lastPrintTime = millis();
    }

    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input == "l") {
            turnLeft90(mpuState, bearingState);
        } else if (input == "r") {
            turnRight90(mpuState, bearingState);
        } else if (input == "180") {
            turn180(mpuState, bearingState);
        } else if (input == "fix") {
            alignToBearing(mpuState, bearingState, bearingState.currentRelativeBearing);
            Serial.print("Fixed to relative bearing: ");
            Serial.println(bearingState.currentRelativeBearing);
        } else if (input == "f") {
            Forward25(mpuState, bearingState, motorState, encoderState);
        } else {
            Serial.println("Invalid command. Please enter 'l', 'r', '180', 'fix', or 'f'.");
        }
    }

    delay(100);
    // Forward25(mpuState, bearingState, motorState, encoderState);
    // delay(1000);
}