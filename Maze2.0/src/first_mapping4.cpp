#include <EEPROM.h>
#include <motor_functions.h>

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

void backtrack_and_reorient()
{
    Serial.println("backtrack_and_reorient starting.");
    turn_180(mpuState, bearingState);
    delay(5000);

    for (count -= 1; count > junction_nodes[index]; count--)
    {
        Serial.print("Count: ");
        Serial.println(count);

        if (movement_arr[count] == 'F')
        {
            Serial.println("Move forward now");
            Forward25(mpuState, bearingState, motorState, encoderState);
        }
        else if (movement_arr[count] == 'L')
        {
            Serial.println("Turn right now");
            turn_right_90(mpuState, bearingState);
        }
        else if (movement_arr[count] == 'R')
        {
            Serial.println("Turn left now");
            turn_left_90(mpuState, bearingState);
        }
    }

    // offset comepensation for front moevement from junction node only
    if (movement_arr[junction_nodes[index]] == 'F')
    {
        Serial.println("Move forward now");
        Forward25(mpuState, bearingState, motorState, encoderState);
    }

    // Reorientation
    if (movement_arr[count] == 'F')
    {
        Serial.println("Reorienting: F (Forward)");
        Serial.println("Turn 180 now");
        turn_180(mpuState, bearingState);
        junction_visited[index] = 1;
        Serial.println("Junction visited stored as 1.");
    }
    else if (movement_arr[count] == 'L')
    {
        Serial.println("Reorienting: L (Left)");
        Serial.println("Turn left now");
        turn_left_90(mpuState, bearingState);
        junction_visited[index] = 2;
        Serial.println("Junction visited stored as 2.");
    }
    else if (movement_arr[count] == 'R')
    {
        Serial.println("Reorienting: R (Right)");
        Serial.println("Turn right now");
        turn_right_90(mpuState, bearingState);
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

    if ((front == -1 && left == -1) || (front == -1 && right == -1))
    {
        if (front == -1 && left == -1 && right == -1)
        {
            Serial.println("End of maze reached.");
            movement_arr[count] = '\0';
            // is_LeBron_done = true;
            flags.is_LeBron_done = 1;
            return;
        }
        return;
    }

    if (front != 0 && (count != junction_nodes[index] || junction_visited[index] < 1)) // front has space
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
        Forward25(mpuState, bearingState, motorState, encoderState);
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else if (left != 0 && (count != junction_nodes[index] || junction_visited[index] < 2)) // left has space
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
        turn_left_90(mpuState, bearingState);
        Serial.println("Turn left 90° done.");
        movement_arr[count] = 'L';
        count++;
        Serial.println("Left turn stored.");

        Serial.println("Move forward now.");
        Forward25(mpuState, bearingState, motorState, encoderState);
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else if (right != 0 && (count != junction_nodes[index] || junction_visited[index] < 3)) // right has space
    {
        Serial.println("Right space detected");

        Serial.println("Turn right now");
        turn_right_90(mpuState, bearingState);
        Serial.println("Turn right 90° done.");
        movement_arr[count] = 'R';
        count++;
        Serial.println("Right turn stored.");

        Serial.println("Move forward now.");
        Forward25(mpuState, bearingState, motorState, encoderState);
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else
    {
        if ((junction_visited[index] == 2 && right == 0) || junction_visited[index] == 3)
        {
            Serial.println("All routes explored, removing junction.");
            junction_visited[index] = 0;
            index--;
        }

        backtrack_and_reorient();
        Serial.println("backtrack_and_reorienting complete.");
    }
}

void init_arrays()
{
    Serial.println("Initializing arrays.");

    memset(movement_arr, 0, sizeof(movement_arr));
    memset(junction_nodes, 0, sizeof(junction_nodes));
    memset(junction_visited, 0, sizeof(junction_visited));

    // memoryReset();
    // Serial.println("EEPROM reseted.");
}

void setup()
{
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

    init_arrays();
}

void loop()
{
    static unsigned long lastPrintTime = 0;
    const unsigned long printInterval = 500;

    updateMPU(mpuState);

    if (millis() - lastPrintTime >= printInterval) {
        printCurrentBearing(mpuState, bearingState);
        lastPrintTime = millis();
    }

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