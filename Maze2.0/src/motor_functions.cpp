#include "motor_functions_header.h"

BooleanFlags flags = {0, 0, 1, 0, 0};
EncoderState encoderState;
MotorState motorState;
MPUState mpuState;
BearingState bearingState;

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

        // int distance = flags.reduce_forward_dist_after_turn == 1 ? 15 : 25;

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
    // flags.reduce_forward_dist_after_turn = 0;
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
    // flags.reduce_forward_dist_after_turn = 1;
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
    // flags.reduce_forward_dist_after_turn = 1;
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
