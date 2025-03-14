#include <EEPROM.h>
#include "motor_functions_header.h"
// #include "og_motor_functions_header.h"

char movement_arr[SIZE];
// int junction_nodes[SIZE];
// int junction_visited[SIZE];
// int index = 0;
int count = 0;
bool is_LeBron_done = false;

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

    // EEPROM.put(addr, junction_nodes);
    // addr += sizeof(junction_nodes);

    // EEPROM.put(addr, index);

    Serial.println("EEPROM write complete.");
}

int memoryRead() 
{
    Serial.println("Executing memoryRead()");
    int addr = 0;
    int check = 0;

    if (EEPROM.get(addr, check) == 0xFFFF) 
    {
        Serial.println("EEPROM is empty.");
        return -1;  // EEPROM is empty
    }

    EEPROM.get(addr, movement_arr);
    addr += sizeof(movement_arr);

    // EEPROM.get(addr, junction_nodes);
    // addr += sizeof(junction_nodes);

    // EEPROM.get(addr, index);

    Serial.println("EEPROM read complete.");
    return 0;
}

void you_who_seek_an_end_to_love_love_will_yield_to_business()
{
    // Serial.println("LeBron is following previously saved movement array.");
    
    if (movement_arr[count] == 'F')
    {
        // Serial.println("Move forward now.");
        Forward25(mpuState, bearingState, motorState, encoderState);
        // Forward25(100);
    }
    else if (movement_arr[count] == 'L')
    {
        // Serial.println("Turn right now.");
        turn_right_90(mpuState, bearingState);
        // turn_right_90();
    }
    else if (movement_arr[count] == 'R')
    {
        // Serial.println("Turn left now.");
        turn_left_90(mpuState, bearingState);
        // turn_left_90();
    }
    else if (movement_arr[count] == '\0')
    {
        Serial.println("End of maze reached.");
        flags.is_LeBron_done = 1;
    }
    count++;
}

void but_you_been_so_outta_touchtouchtouchtouchtouch()
{
    Serial.println("But you been so outta touch touch touch touch touch.");
    Serial.println("Thought about you way too much much much much much.");
    
    memset(movement_arr, 0, sizeof(movement_arr));
    // memset(junction_nodes, 0, sizeof(junction_nodes));
    // memset(junction_visited, 0, sizeof(junction_visited));
    
    memoryRead();
    Serial.println("-----");
    Serial.println(movement_arr);
    Serial.println("-----");
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
    // calculateError();
    
    encoderState.leftTotalDistance = 0.0;
    encoderState.rightTotalDistance = 0.0;
    motorState.targetReached = false;

    delay(1000);
    // updateMPU();
    // initialYaw = yaw;
    updateMPU(mpuState);
    mpuState.initialYaw = mpuState.yaw;
    bearingState.currentRelativeBearing = 0.0;

    but_you_been_so_outta_touchtouchtouchtouchtouch();
}

void loop()
{
    if (flags.is_LeBron_done == 0)
    {
        you_who_seek_an_end_to_love_love_will_yield_to_business();
    }
}