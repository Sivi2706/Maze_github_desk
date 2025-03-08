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

// Motor Pins
#define IN1 2
#define IN2 4
#define IN3 5
#define IN4 6
#define FNA 3
#define FNB 11

// Maze dimensions
#define ROWS 8
#define COLS 8
#define MAX_GOONS ROWS * COLS

// Ultrasonic Sensor Functions
void ultrasonicSetup(int trigPin, int echoPin) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.println("Ultrasonic sensor initialized.");
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
    float distance = getDistance(trigPin, echoPin);
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    return distance <= 10 ? 0 : 1; // Threshold distance for junction detection
}

// Motor Control Functions
void MoveForward(int PWM) {
    Serial.println("Executing MoveForward()");
    analogWrite(FNA, PWM);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, PWM);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving Forward...");
}

void stopMotors() {
    Serial.println("Executing stopMotors()");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stopped.");
}

void turnRight90() {
    Serial.println("Executing turnRight90()");
    analogWrite(FNA, 100);  // Left motor forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, 100);  // Right motor backward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(500);  // Adjust delay for 90° turn
    stopMotors();
    Serial.println("Turned Right 90°");
}

void turnLeft90() {
    Serial.println("Executing turnLeft90()");
    analogWrite(FNA, 100);  // Left motor backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 100);  // Right motor forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(500);  // Adjust delay for 90° turn
    stopMotors();
    Serial.println("Turned Left 90°");
}

void turn180() {
    Serial.println("Executing turn180()");
    analogWrite(FNA, 100);  // Left motor backward
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(FNB, 100);  // Right motor forward
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(1000);  // Adjust delay for 180° turn
    stopMotors();
    Serial.println("Turned 180°");
}

// Maze Navigation Algorithm
int back_it_up_LeBron(int back_index, char* temp_movement, int* movement_index, int* index_counter) {
    Serial.println("Executing back_it_up_LeBron()");
    turn180();  // Turn away from the wall

    // Backtrack to the last junction
    for (int i = back_index; i > movement_index[*index_counter]; i--) {
        Serial.print("Backtracking step: ");
        Serial.println(temp_movement[i]);
        if (temp_movement[i] == 'F') {
            MoveForward(100);
        } else if (temp_movement[i] == 'L') {
            turnRight90();  // Opposite direction
        } else if (temp_movement[i] == 'R') {
            turnLeft90();   // Opposite direction
        }
    }

    // Return to the most recent junction
    Serial.println("Backtracking complete.");
    return movement_index[*index_counter];
}

void keep_going_LeBron(char* temp_movement, int* movement_index, int* junction_gooned, int* index_counter, int i) {
    Serial.println("Executing keep_going_LeBron()");
    int front, left, right;

    for (i = i; i < MAX_GOONS; i++) {
        front = checkTheDih(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
        left = checkTheDih(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
        right = checkTheDih(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

        if (front == -1 && left == -1 && right == -1) {  // End of maze
            Serial.println("End of maze reached.");
            temp_movement[i] = '\0';  // Null terminate the array
            break;
        } else if (junction_gooned[*index_counter] < 1 && front) {  // Forward path
            if (left || right) movement_index[++(*index_counter)] = i;
            MoveForward(255);
            temp_movement[i] = 'F';
            Serial.println("Movement: F (Forward)");
        } else if (junction_gooned[*index_counter] < 2 && left) {  // Left path
            if (right) movement_index[++(*index_counter)] = i;
            turnLeft90();
            temp_movement[i] = 'L';
            MoveForward(255);
            temp_movement[i++] = 'F';
            Serial.println("Movement: L (Left)");
        } else if (junction_gooned[*index_counter] < 3 && right) {  // Right path
            turnRight90();
            temp_movement[i] = 'R';
            MoveForward(255);
            temp_movement[i++] = 'F';
            Serial.println("Movement: R (Right)");
        } else {  // Dead end or backtrack
            Serial.println("Dead end detected. Backtracking...");
            i = back_it_up_LeBron(i, temp_movement, movement_index, index_counter);

            if (temp_movement[i + 1] == 'F') {
                turn180();
                junction_gooned[*index_counter] = 1;
                Serial.println("Reorienting: F (Forward)");
            } else if (temp_movement[i + 1] == 'L') {
                turnLeft90();
                junction_gooned[*index_counter] = 2;
                Serial.println("Reorienting: L (Left)");
            } else if (temp_movement[i + 1] == 'R') {
                turnRight90();
                junction_gooned[*index_counter] = 3;
                Serial.println("Reorienting: R (Right)");
            }
        }
    }
}

int follow_gooning_path(char* temp_movement, int* movement_index, int* index_counter) {
    Serial.println("Executing follow_gooning_path()");
    int i;
    for (i = 0; i < movement_index[*index_counter - 1] + 1; i++) {
        Serial.print("Following path step: ");
        Serial.println(temp_movement[i]);
        if (temp_movement[i] == 'F') {
            MoveForward(100);
        } else if (temp_movement[i] == 'L') {
            turnLeft90();
            MoveForward(100);
        } else if (temp_movement[i] == 'R') {
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

void start_gooning() {
    Serial.println("Executing start_gooning()");
    char temp_movement[MAX_GOONS];
    char final_movement[MAX_GOONS];
    int movement_index[MAX_GOONS];
    int junction_gooned[MAX_GOONS];
    int index_counter = 0;
    int aura = 0;

    memset(temp_movement, 0, sizeof(temp_movement));
    memset(final_movement, 0, sizeof(final_movement));
    memset(movement_index, 0, sizeof(movement_index));
    memset(junction_gooned, 0, sizeof(junction_gooned));

    if (memoryRead(final_movement, movement_index, &index_counter) < 0) {
        Serial.println("No saved path found. Exploring new path...");
        keep_going_LeBron(temp_movement, movement_index, junction_gooned, &index_counter, aura);
    } else {
        Serial.println("Saved path found. Following path...");
        memcpy(final_movement, temp_movement, sizeof(final_movement));
        aura = follow_gooning_path(final_movement, movement_index, &index_counter);
        keep_going_LeBron(temp_movement, movement_index, junction_gooned, &index_counter, aura);

        if (strlen(temp_movement) < strlen(final_movement)) {
            Serial.println("New shorter path found. Updating EEPROM...");
            memoryReset();
            memoryWrite(temp_movement, movement_index, &index_counter);
        }
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
    ultrasonicSetup(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    ultrasonicSetup(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    ultrasonicSetup(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);
    Serial.println("Setup complete.");
}

void loop() {
    start_gooning();
}