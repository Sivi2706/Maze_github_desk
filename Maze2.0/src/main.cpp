#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <EEPROM.h>

#define ROWS 8
#define COLS 8
#define SIZE ROWS * COLS

char movement_arr[SIZE];
int junction_nodes[SIZE];
int junction_visited[SIZE];
int index = 0;
int count = 0;

int checkDist()
{
    // this function should 
    // return 0 if there is no space in that direction
    // return 1 if there is space in that direction
    // return -1 if the space is more than 200 cm
    Serial.println("Distance checked");
}

void move_forward()
{
    Serial.println("Moving forward.");

    for (int i = 0; i < 5; i++)
    {
        Serial.println(i);
    }

    delay(10000);
}

void turn_left_90()
{
    Serial.println("Turning left 90°.");

    for (int i = 0; i < 5; i++)
    {
        Serial.println(i);
    }

    delay(10000);
}

void turn_right_90()
{
    Serial.println("Turning right 90°.");

    for (int i = 0; i < 5; i++)
    {
        Serial.println(i);
    }

    delay(10000);
}

void turn_180()
{
    Serial.println("Turning 180°.");

    for (int i = 0; i < 5; i++)
    {
        Serial.println(i);
    }

    delay(10000);
}

void backtrack()
{
    turn_180();

    Serial.println("Backtrack starting.");

    for (; count > junction_nodes[index]; count--)
    {
        if (movement_arr[count] == 'F') move_forward();
        else if (movement_arr[count] == 'L') turn_right_90();
        else if (movement_arr[count] == 'R') turn_left_90();
    }    
}

void search_maze()
{
    Serial.println("Searching maze.");
    Serial.print("Currently in loop: "); Serial.println(count);

    int front = 0, left = 0, right = 0;

    front = checkDist();
    left = checkDist();
    right = checkDist();

    Serial.print("Front: "); Serial.println(front); 
    Serial.print("Left: "); Serial.println(left);
    Serial.print("Right: "); Serial.println(right);

    if (front == 1 && junction_nodes[index] < 1)                 // front has space
    {
        Serial.println("Front space detected");
        if (left == 1 || right == 1)
        {
            Serial.println("Left and right space detected");
            index++;
            junction_nodes[index] = count;
            Serial.println("Junction node stored.");
        }
       
        move_forward();
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else if (left == 1 && junction_nodes[index] < 2)             // left has space
    {
        Serial.println("Left space detected");
        if (right == 1)
        {
            Serial.println("Right space detected");
            index++;
            junction_nodes[index] = count;
            Serial.println("Junction node stored.");
        }
        
        turn_left_90();
        Serial.println("Turn left 90° done.");
        movement_arr[count] = 'L';
        count++;
        Serial.println("Left turn stored.");
        
        move_forward();
        Serial.println("Forward movement done.");
        movement_arr[count] = 'F';
        count++;
        Serial.println("Forward movement stored.");
    }
    else if (right == 1 && junction_nodes[index] < 3)           // right has space
    {
        Serial.println("Right space detected");
        
        turn_right_90();
        Serial.println("Turn right 90° done.");
        movement_arr[count] = 'R';
        count++;
        Serial.println("Right turn stored.");
        
        move_forward();
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

        backtrack();
        Serial.println("Backtracking complete.");

        if (movement_arr[count + 1] == 'F')
        {
            turn_180();
            Serial.println("Reorienting: F (Forward)");
            junction_visited[index] = 1;
            Serial.println("Junction visited stored as 1.");
        }
        else if (movement_arr[count + 1] == 'L')
        {
            turn_right_90();
            Serial.println("Reorienting: L (Left)");
            junction_visited[index] = 2;
            Serial.println("Junction visited stored as 2.");
        }
        else if (movement_arr[count + 1] == 'R')
        {
            turn_left_90();
            Serial.println("Reorienting: R (Right)");
            junction_visited[index] = 3;
            Serial.println("Junction visited stored as 3.");
        }
    }
}

void init_arrays()
{
    Serial.println("Initializing arrays.");

    memset(movement_arr, 0, sizeof(movement_arr));
    memset(junction_nodes, 0, sizeof(junction_nodes));
    memset(junction_visited, 0, sizeof(junction_visited));
}

void setup() 
{
    Serial.begin(9600);

    init_arrays();
}

void loop()
{
    search_maze();
}