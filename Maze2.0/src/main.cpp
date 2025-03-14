#include <EEPROM.h>
#include "improved_motor.h"

char movement_arr[SIZE];
uint8_t junction_nodes[SIZE];
uint8_t junction_visited[SIZE];
uint8_t index = 0;
uint8_t count = 0;

void memoryReset() {
    Serial.println(F("Executing memoryReset()"));
    for (unsigned int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);
    }
    Serial.println(F("EEPROM reset complete"));
}

void memoryWrite() {
    Serial.println(F("Executing memoryWrite()"));
    int addr = 0;

    EEPROM.put(addr, movement_arr);
    addr += sizeof(movement_arr);

    // EEPROM.put(addr, junction_nodes);
    // addr += sizeof(junction_nodes);

    // EEPROM.put(addr, index);

    Serial.println(F("EEPROM write complete"));
}

int memoryRead() {
    Serial.println(F("Executing memoryRead()"));
    int addr = 0;
    if (EEPROM.read(0) != 0) {
        Serial.println(F("EEPROM is empty"));
        return -1;
    }

    EEPROM.get(addr, movement_arr);
    addr += sizeof(movement_arr);

    // EEPROM.get(addr, junction_nodes);
    // addr += sizeof(junction_nodes);

    // EEPROM.get(addr, index);

    Serial.println(F("EEPROM read complete"));
    return 0;
}

void backtrack_and_reorient() {
    Serial.println(F("Backtrack_and_reorient starting"));
    turn180(mpuState, bearingState);
    delay(3000);

    for (count -= 1; count > junction_nodes[index]; count--) {
        Serial.println(F("Count: "));
        
        if (movement_arr[count] == 'F') {
            Serial.println(F("Move forward now"));
            moveForwards(90, mpuState, bearingState, motorState, encoderState);
        } else if (movement_arr[count] == 'L') {
            Serial.println(F("Turn right now"));
            turnRight90(mpuState, bearingState);
        } else if (movement_arr[count] == 'R') {
            Serial.println(F("Turn left now"));
            turnLeft90(mpuState, bearingState);
        }
    }

    if (movement_arr[junction_nodes[index]] == 'F') {
        Serial.println(F("Move forwards now"));
        moveForwards(90, mpuState, bearingState, motorState, encoderState);
    }

    if (movement_arr[count] == 'F') {
        Serial.println(F("Turn 180 now"));
        turn180(mpuState, bearingState);
        junction_visited[index] = 1;
    } else if (movement_arr[count] == 'L') {
        Serial.println(F("Turn left now"));
        turnLeft90(mpuState, bearingState);
        junction_visited[index] = 2;
    } else if (movement_arr[count] == 'R') {
        Serial.println(F("Turn right now"));
        turnRight90(mpuState, bearingState);
        junction_visited[index] = 3;
    }
}

void search_maze() {
    Serial.print(F("Currently in loop: "));
    Serial.println(count);

    int front, left, right;

    front = checkDist(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    left = checkDist(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    right = checkDist(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

    Serial.print(F("FRONT: "));
    Serial.println(front);
    Serial.print(F("LEFT: "));
    Serial.println(left);
    Serial.print(F("RIGHT: "));
    Serial.println(right);
    Serial.println(F("------"));
    Serial.println(movement_arr);
    Serial.println(F("------"));

    delay(3000);

    // if ((front == -1 && left == -1) || (front == -1 && right == -1)) {
    //     if (front == -1 && left == -1 && right == -1) {
    //         Serial.println(F("End of maze reached"));
    //         movement_arr[count] = '\0';
    //         flags.is_LeBron_done = 1;
    //         return;
    //     }
    //     return;
    // }

    if (front != 0 && (count != junction_nodes[index] || junction_visited[index] < 1)) {
        if (left == 1 || right == 1) {
            index++;
            junction_nodes[index] = count;
        }

        Serial.println(F("Move forward now 1"));
        moveForwards(90, mpuState, bearingState, motorState, encoderState);
        movement_arr[count] = 'F';
        count++;
    } else if (left != 0 && (count != junction_nodes[index] || junction_visited[index] < 2)) {
        if (right == 1) {
            index++;
            junction_nodes[index] = count;
        }

        Serial.println(F("Turn left now"));
        turnLeft90(mpuState, bearingState);
        movement_arr[count] = 'L';
        count++;

        Serial.println(F("Move forward now 2"));
        moveForwards(90, mpuState, bearingState, motorState, encoderState);
        movement_arr[count] = 'F';
        count++;
    } else if (right != 0 && (count != junction_nodes[index] || junction_visited[index] < 3)) {
        Serial.println(F("Turn right now"));
        turnRight90(mpuState, bearingState);
        movement_arr[count] = 'R';
        count++;
        
        Serial.println(F("Move forward now 3"));
        moveForwards(90, mpuState, bearingState, motorState, encoderState);
        movement_arr[count] = 'F';
        count++;
    } else {
        if ((junction_visited[index] == 2 && right == 0) || junction_visited[index] == 3) {
            Serial.println(F("All routes explored, removing junction"));
            junction_visited[index] = 0;
            index--;
        }

        backtrack_and_reorient();
    }
}

void init_arrays() {
    memset(movement_arr, 0, sizeof(movement_arr));
    memset(junction_nodes, 0, sizeof(junction_nodes));
    memset(junction_visited, 0, sizeof(junction_visited));

    // memoryReset();
    // Serial.println("EEPROM RESETED");
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

    Serial.print(F("Initial yaw set to: "));
    Serial.println(mpuState.initialYaw);
    Serial.println(F("Starting with relative bearing of 0 degrees"));
}

void loop()
{
    // static unsigned long lastPrintTime = 0;
    // const unsigned long printInterval = 500;

    // updateMPU(mpuState);

    // if (millis() - lastPrintTime >= printInterval) {
    //     printCurrentBearing(mpuState, bearingState);
    //     lastPrintTime = millis();
    // }

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