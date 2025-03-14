#include "improved_motor.h"

BooleanFlags flags = {0, 0, 1, 0, 0, 0};
EncoderState encoderState;
MPUState mpuState;
BearingState bearingState;
MotorState motorState;

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
            moveForwards(100, mpuState, bearingState, motorState, encoderState);
        } else {
            alignToBearing(mpu, bearing, bearing.currentRelativeBearing);
        }
    }
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
            int correctionPWM = min(abs(error) * 20, 50); // Proportional correction factor //change the value of abs(error) * VAR 

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

        if (flags.has_LeBron_turn == 1) {
            encoderState.target = 5;
        } else {encoderState.target = 25;}

        Serial.print(F("Niama distance is: "));
        Serial.println(encoderState.target);
        // Stop if the target distance is reached
        if (avgDistance >= encoderState.target) {
            stopMotors();
            motor.targetReached = true;
            Serial.println("Target distance of 25 cm reached. Stopping.");
            Serial.print(F("Target distance of "));
            Serial.print(encoderState.target);
            Serial.println(F("reached"));
            break; // Exit the loop immediately
        }

        delay(5); // Small delay to avoid overloading the loop
    }
    maintainBearing(mpu, bearing, motor);

    // Stop motors when the target is reached or movement is interrupted
    stopMotors();
    flags.has_LeBron_turn = 0;
    encoder.leftTotalDistance = encoder.rightTotalDistance = encoder.leftPulses = encoder.rightPulses = encoder.target = 0;
    motor.targetReached = false;
    Serial.println("Forward movement complete.");
}

void improvedForwards(int PWM, MPUState &mpu, BearingState &bearing, MotorState &motor, EncoderState &encoderState) {
    if (!motor.isMovingForward && !motor.targetReached) {
        motor.targetReached = false;
    }

    float targetAbsoluteBearing = mpu.yaw;
}

void turnLeft90(MPUState &mpu, BearingState &bearing) {
    flags.has_LeBron_turn = 1;
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
    analogWrite(FNA, 100 * LEFT_MOTOR_CALIBRATION);  // Adjusted to match turnRight90
    analogWrite(FNB, 100 * RIGHT_MOTOR_CALIBRATION); // Adjusted to match turnRight90
    
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
    encoderState.leftPulses = encoderState.rightPulses = encoderState.leftTotalDistance = encoderState.rightTotalDistance = encoderState.target = 0;
    Serial.println("Alignment complete. Robot is at target bearing.");
}

//================================

void turnRight90(MPUState &mpu, BearingState &bearing) {
    flags.has_LeBron_turn = 1;
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
    analogWrite(FNA, 100 * LEFT_MOTOR_CALIBRATION);
    analogWrite(FNB, 100 * RIGHT_MOTOR_CALIBRATION);
    
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
    encoderState.rightPulses = encoderState.leftPulses = encoderState.rightTotalDistance = encoderState.leftTotalDistance = encoderState.target = 0;
    Serial.println("Alignment complete. Robot is at target bearing.");
}

void turn180(MPUState &mpu, BearingState &bearing) {
    float front = getDistance(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);

    if (front < 4) {
        reverse(100, 4 - front, mpuState, bearingState, motorState, encoderState);
        Serial.println(F("Finished reversing before 180"));
    }

    float newRelativeBearing = bearing.currentRelativeBearing + 180;
    if (newRelativeBearing >= 360) newRelativeBearing -= 360;

    Serial.print("Turning 180° from relative bearing ");
    Serial.print(bearing.currentRelativeBearing);
    Serial.print("° to ");
    Serial.print(newRelativeBearing);
    Serial.println("°");

    analogWrite(FNA, 100 * LEFT_MOTOR_CALIBRATION);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 100 * RIGHT_MOTOR_CALIBRATION);
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
    encoderState.rightPulses = encoderState.leftPulses = encoderState.rightTotalDistance = encoderState.leftTotalDistance = encoderState.target = 0;
    Serial.println("Alignment complete. Robot is at target bearing.");
}

void reverse(int PWM, float distance, MPUState &mpu, BearingState &bearing, MotorState &motor, EncoderState &encoder) {
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
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    Serial.println("Reverse...");

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
            int correctionPWM = min(abs(error) * 20, 50); // Proportional correction factor //change the value of abs(error) * VAR 

            if (error > 0) {
                // Turn slightly left (reduce left motor speed)
                analogWrite(FNA, (PWM * LEFT_MOTOR_CALIBRATION) + correctionPWM);
                analogWrite(FNB, (PWM * RIGHT_MOTOR_CALIBRATION) - correctionPWM);
            } else {
                // Turn slightly right (reduce right motor speed)
                analogWrite(FNA, (PWM * LEFT_MOTOR_CALIBRATION) - correctionPWM);
                analogWrite(FNB, (PWM * RIGHT_MOTOR_CALIBRATION) + correctionPWM);
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

        // if (flags.has_LeBron_turn == 1) {
        //     encoderState.target = 5;
        // } else {encoderState.target = 25;}
        encoderState.target = distance;
        // Stop if the target distance is reached
        if (avgDistance >= encoderState.target) {
            stopMotors();
            motor.targetReached = true;
            Serial.println("Target distance of 25 cm reached. Stopping.");
            Serial.print(F("Target distance of "));
            Serial.print(encoderState.target);
            Serial.println(F("reached"));
            break; // Exit the loop immediately
        }

        delay(5); // Small delay to avoid overloading the loop
    }
    maintainBearing(mpu, bearing, motor);

    // Stop motors when the target is reached or movement is interrupted
    stopMotors();
    flags.has_LeBron_turn = 0;
    encoder.leftTotalDistance = encoder.rightTotalDistance = encoder.leftPulses = encoder.rightPulses = encoder.target = 0;
    motor.targetReached = false;
    Serial.println("Forward movement complete.");
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

        if (flags.has_LeBron_turn == 1) encoder.target = 5;
        else encoder.target = 25;

        if (avgDistance >= encoder.target && !motor.targetReached) {
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



// void loop() {
//     static unsigned long lastPrintTime = 0;
//     const unsigned long printInterval = 500;

//     updateMPU(mpuState);

//     if (millis() - lastPrintTime >= printInterval) {
//         printCurrentBearing(mpuState, bearingState);
//         lastPrintTime = millis();
//     }

//     if (Serial.available() > 0) {
//         String input = Serial.readStringUntil('\n');
//         input.trim();

//         if (input == "l") {
//             turnLeft90(mpuState, bearingState);
//         } else if (input == "r") {
//             turnRight90(mpuState, bearingState);
//         } else if (input == "180") {
//             turn180(mpuState, bearingState);
//         } else if (input == "fix") {
//             alignToBearing(mpuState, bearingState, bearingState.currentRelativeBearing);
//             Serial.print("Fixed to relative bearing: ");
//             Serial.println(bearingState.currentRelativeBearing);
//         } else if (input == "f") {
//             moveForwards(100, mpuState, bearingState, motorState, encoderState);
//         } else {
//             Serial.println("Invalid command. Please enter 'l', 'r', '180', 'fix', or 'f'.");
//         }
//     }

//     delay(100);
//     // Forward25(mpuState, bearingState, motorState, encoderState);
//     // delay(1000);
// }

// void loop()
// {
//     // static unsigned long lastPrintTime = 0;
//     // const unsigned long printInterval = 500;

//     // updateMPU(mpuState);

//     // if (millis() - lastPrintTime >= printInterval) {
//     //     printCurrentBearing(mpuState, bearingState);
//     //     lastPrintTime = millis();
//     // }

//     if (flags.is_LeBron_done == 0) 
//     {
//         search_maze();
//     }
//     else
//     {
//         if (flags.has_LeBron_written == 0)
//         {
//             memoryWrite();
//             flags.has_LeBron_written = true;
//         }
//     }
// }