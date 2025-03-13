#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <EEPROM.h>

#define ROWS 8
#define COLS 8
#define SIZE ROWS * COLS

// // Ultrasonic Sensor Pins
// #define FRONT_TRIGGER_PIN 7
// #define FRONT_ECHO_PIN 8
// #define LEFT_TRIGGER_PIN A3
// #define LEFT_ECHO_PIN A2
// #define RIGHT_TRIGGER_PIN A0
// #define RIGHT_ECHO_PIN A1

// // Motor Pins
// #define IN1 2
// #define IN2 4
// #define IN3 5
// #define IN4 6
// #define FNA 3
// #define FNB 11

char movement_arr[SIZE];
// int junction_nodes[SIZE];
// int junction_visited[SIZE];
// int index = 0;
int count = 0;
bool is_LeBron_done = false;

void move_forward()
{
    Serial.println("Moving forward.");

    for (int i = 0; i < 5; i++)
    {
        Serial.println(i);
    }

    delay(7000);
}

void turn_left_90()
{
    Serial.println("Turning left 90°.");

    for (int i = 0; i < 5; i++)
    {
        Serial.println(i);
    }

    delay(7000);
}

void turn_right_90()
{
    Serial.println("Turning right 90°.");

    for (int i = 0; i < 5; i++)
    {
        Serial.println(i);
    }

    delay(7000);
}

void turn_180()
{
    Serial.println("Turning 180°.");

    for (int i = 0; i < 5; i++)
    {
        Serial.println(i);
    }

    delay(7000);
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
    Serial.println("LeBron is following previously saved movement array.");
    
    if (movement_arr[count] == 'F')
    {
        Serial.println("Move forward now.");
        move_forward();
    }
    else if (movement_arr[count] == 'L')
    {
        Serial.println("Turn right now.");
        turn_right_90();
    }
    else if (movement_arr[count] == 'R')
    {
        Serial.println("Turn left now.");
        turn_left_90();
    }
    else if (movement_arr[count] == '\0')
    {
        Serial.println("End of maze reached.");
        is_LeBron_done = true;
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

    // pinMode(IN1, OUTPUT);
    // pinMode(IN2, OUTPUT);
    // pinMode(IN3, OUTPUT);
    // pinMode(IN4, OUTPUT);
    // pinMode(FNA, OUTPUT);
    // pinMode(FNB, OUTPUT);
    // ultrasonicSetup(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    // ultrasonicSetup(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    // ultrasonicSetup(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);
    // Serial.println("Setup complete.");

    but_you_been_so_outta_touchtouchtouchtouchtouch();
}

void loop()
{
    if (!is_LeBron_done)
    {
        you_who_seek_an_end_to_love_love_will_yield_to_business();
    }
}