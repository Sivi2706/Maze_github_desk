#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <EEPROM.h>

#define ROWS 8
#define COLS 8
#define SIZE ROWS * COLS

// Ultrasonic Sensor Pins
#define FRONT_TRIGGER_PIN 7
#define FRONT_ECHO_PIN 8
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

char final_arr[SIZE];
char movement_arr[SIZE];
int junction_nodes[SIZE];
int junction_visited[SIZE];
int index = 0;
int node_temp = 0;
int count = 0;
bool is_LeBron_done = false;
bool LeBron_finished_following = false;
bool has_LeBron_written = false;

void ultrasonicSetup(int trigPin, int echoPin)
{
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

float getDistance(int trigPin, int echoPin)
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long duration = pulseIn(echoPin, HIGH, 30000);
    return (duration == 0) ? 0 : (duration * 0.0343 / 2.0);
}

int checkDist(int trigPin, int echoPin)
{
    // this function should
    // return 0 if there is no space in that direction
    // return 1 if there is space in that direction
    // return -1 if the space is more than 200 cm
    float distance = getDistance(trigPin, echoPin);

    Serial.print("Distance: ");
    Serial.println(distance);

    if (distance > 75)
        return -1;
    else if (distance <= 10)
        return 0;
    else
        return 1;

    Serial.println("Distance checked");
}

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

    EEPROM.put(addr, junction_nodes);
    addr += sizeof(junction_nodes);

    EEPROM.put(addr, index);

    Serial.println("EEPROM write complete.");
}

int memoryRead() 
{
    Serial.println("Executing memoryRead()");
    int addr = 0;
    byte check = 0;
    EEPROM.get(addr, check);

    if(EEPROM.read(0) != 0)
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

uint8_t arrLen(char* lebroooooooooooooon)
{
    uint8_t len = 0;
    while (lebroooooooooooooon[len]!= '\0')
    {
        len++;
    }
    return len;
}

void backtrack_and_reorient()
{
    Serial.println("Backtrack starting.");
    turn_180();
    delay(5000);

    for (count -= 1; count > junction_nodes[index]; count--)
    {
        Serial.print("Count: ");
        Serial.println(count);

        if (movement_arr[count] == 'F')
        {
            Serial.println("Move forward now");
            move_forward();
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

    // offset for front movement from junction node only
    if (movement_arr[junction_nodes[index]] == 'F')
    {
        Serial.println("Move forward now");
        move_forward();
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

    if (front == -1 && left == -1 && right == -1)
    {
        Serial.println("End of maze reached.");
        movement_arr[count] = '\0';
        is_LeBron_done = true;
        return;
    }

    // allow movement if:
    // i) front has space and LeBron is not at the same junction 
    // ii) front has space and front route has not been visited
    // logic is that if it is not the same junction, then the front route has not been visited yet or smtg liddat
    // if in the previously saved movement_arr is of the same movement type,
    // dont bother going in this direction, backtrack to previous junction, skip current loop
    if (front != 0 && (count != junction_nodes[index] || junction_visited[index] < 1) 
                    && final_arr[count] != 'F')
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
        move_forward();
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else if (left != 0 && (count != junction_nodes[index] || junction_visited[index] < 2) 
                        && final_arr[count] != 'L') 
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
        move_forward();
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else if (right != 0 && (count != junction_nodes[index] || junction_visited[index] < 3)
                        && final_arr[count] != 'R') 
    {
        Serial.println("Right space detected");

        Serial.println("Turn right now");
        turn_right_90();
        Serial.println("Turn right 90° done.");
        movement_arr[count] = 'R';
        count++;
        Serial.println("Right turn stored.");

        Serial.println("Move forward now.");
        move_forward();
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else // dead end
    {
        // all junction visited
        if ((junction_visited[index] == 2 && right == 0) || junction_visited[index] == 3)
        {
            Serial.println("All routes explored, removing junction.");
            junction_visited[index] = 0;
            index--;
        }
        backtrack_and_reorient();
        Serial.println("Backtracking complete.");
    }

    // condition to loop backtrack to 2nd last junction node when current movement_arr is longer than previous one
    if (final_arr[0] != 0 && count > arrLen(final_arr))
    {
        backtrack_and_reorient();
    }
}

void you_who_seek_an_end_to_love_love_will_yield_to_business()
{
    if (count <= junction_nodes[index])
    {
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
            LeBron_finished_following = true;
        }
        count++;
    }
}

void init_arrays()
{
    Serial.println("Initializing arrays.");

    memset(final_arr, 0, sizeof(final_arr));
    memset(movement_arr, 0, sizeof(movement_arr));
    memset(junction_nodes, 0, sizeof(junction_nodes));
    memset(junction_visited, 0, sizeof(junction_visited));

    if (memoryRead() == -1)      // if EEPROM is empty
    {
        Serial.println("EEPROM is empty.");
        LeBron_finished_following = true;
    }
    else
    {
        Serial.println("Movement array copied.");
        memcpy(final_arr, movement_arr, sizeof(movement_arr));
    }
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
    ultrasonicSetup(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    ultrasonicSetup(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    ultrasonicSetup(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);
    Serial.println("Setup complete.");

    init_arrays();
}

void loop()
{
    // search_maze();
    if (!LeBron_finished_following && !is_LeBron_done)
    {
        index -= 1;                              // decrement index to point to a new last junction after every failed shortcut
        node_temp = junction_nodes[index];      // saves second last junction node to loop back to after failed shortcut
        you_who_seek_an_end_to_love_love_will_yield_to_business();
    }
    else if (LeBron_finished_following && !is_LeBron_done)
    {
        search_maze();
    }
    else
    {
        if (!has_LeBron_written)
        {
            if (arrLen(movement_arr) < arrLen(final_arr))
            {
                memoryWrite();
            }
            has_LeBron_written = true;
        }
    }
}