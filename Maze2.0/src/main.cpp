
#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>

// Ultrasonic Sensor Pins
#define FRONT_TRIGGER_PIN 10  // Changed to pin 10
#define FRONT_ECHO_PIN 9      // Changed to pin 9
#define LEFT_TRIGGER_PIN A3
#define LEFT_ECHO_PIN A2
#define RIGHT_TRIGGER_PIN A0
#define RIGHT_ECHO_PIN A1

// Encoder pin definitions
#define LEFT_ENCODER_PIN 13  // Changed from on-board LED to rotary encoder
#define RIGHT_ENCODER_PIN 12

// Motor Pins
#define IN1 2
#define IN2 4
#define IN3 5
#define IN4 6
#define FNA 3
#define FNB 11

// Target distance to travel (in centimeters)
#define TARGET_DISTANCE 90.0

// Encoder and distance calculation variables
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;
const unsigned int PULSES_PER_TURN = 20;
const float WHEEL_DIAMETER = 4;  // in cm
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Separate distance tracking for each wheel
float leftTotalDistance = 0.0;
float rightTotalDistance = 0.0;
bool isMovingForward = false;
bool targetReached = false;

// Interrupt service routines for encoders
void leftEncoderISR() {
    leftPulses++;
}

void rightEncoderISR() {
    rightPulses++;
}


// Variables for RPM calculation
unsigned long lastLeftPulseTime = 0;
unsigned long lastRightPulseTime = 0;
float leftRPM = 0.0;
float rightRPM = 0.0;

// Function to calculate RPM
void calculateRPM() {
    unsigned long currentTime = millis();

    // Calculate RPM for the left encoder
    if (leftPulses > 0) {
        unsigned long timeDiff = currentTime - lastLeftPulseTime;
        leftRPM = (leftPulses / (float)PULSES_PER_TURN) * (60000.0 / timeDiff); // RPM = (pulses per turn) * (ms per minute / time difference)
        lastLeftPulseTime = currentTime;
        leftPulses = 0; // Reset pulse count after calculation
    }

    // Calculate RPM for the right encoder
    if (rightPulses > 0) {
        unsigned long timeDiff = currentTime - lastRightPulseTime;
        rightRPM = (rightPulses / (float)PULSES_PER_TURN) * (60000.0 / timeDiff); // RPM = (pulses per turn) * (ms per minute / time difference)
        lastRightPulseTime = currentTime;
        rightPulses = 0; // Reset pulse count after calculation
    }
}

// Ultrasonic Sensor Functions
void ultrasonicSetup(int trigPin, int echoPin) {
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);
}

// MPU6050 I2C address
const int MPU = 0x68;

// Gyro scale factor
const float GYRO_SCALE = 1.0 / 131.0;

// Variables to hold gyro outputs
float gyroX, gyroY, gyroZ;

// Variables to hold errors for calibration
float GyroErrorX, GyroErrorY, GyroErrorZ;

// Variables to hold angles
float roll, pitch, yaw;

// Timing variables
float elapsedTime, previousTime, currentTime;

// Function to read raw gyro data from MPU6050
void getOrientation() {
    Wire.beginTransmission(MPU);
    Wire.write(0x43); // Start reading from register 0x43 (GYRO_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); // Read 6 bytes (2 bytes per axis)

    // Read raw gyro data for X, Y, and Z axes
    gyroX = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE; // X-axis
    gyroY = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE; // Y-axis
    gyroZ = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE; // Z-axis
}

// Function to calculate gyro errors for calibration
void calculateError() {
    byte c = 0;
    GyroErrorX = 0;
    GyroErrorY = 0;
    GyroErrorZ = 0;

    // Read gyro values 200 times and accumulate errors
    while (c < 200) {
        getOrientation();
        GyroErrorX += gyroX;
        GyroErrorY += gyroY;
        GyroErrorZ += gyroZ;
        c++;
    }

    // Calculate average error for each axis
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;

    Serial.println("Gyroscope calibration complete.");
}

// MPU6050 setup function
void mpuSetup() {
    Wire.begin();                      // Initialize I2C communication
    Wire.beginTransmission(MPU);       // Start communication with MPU6050
    Wire.write(0x6B);                  // Talk to the PWR_MGMT_1 register (6B)
    Wire.write(0x00);                  // Reset the MPU6050
    Wire.endTransmission(true);        // End the transmission

    // Calibrate the gyroscope
    calculateError();
}

// Function to update roll, pitch, and yaw angles
void updateMPU() {
    // Calculate elapsed time
    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) * 0.001; // Convert to seconds

    // Read gyro data
    getOrientation();

    // Correct gyro outputs with calculated error values
    gyroX -= GyroErrorX;
    gyroY -= GyroErrorY;
    gyroZ -= GyroErrorZ;

    // Calculate angles for roll, pitch, and yaw
    roll += gyroX * elapsedTime;
    pitch += gyroY * elapsedTime;
    yaw += gyroZ * elapsedTime;

    // Round angles to 1 decimal place
    roll = round(roll * 10) / 10.0;
    pitch = round(pitch * 10) / 10.0;
    yaw = round(yaw * 10) / 10.0;
}

void setup() {
    Serial.begin(115200);  // Changed baud rate to 115200

    // Motor Pin Setup
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(FNA, OUTPUT);
    pinMode(FNB, OUTPUT);

    // Configure encoder pins
    pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
    pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

    // Initialize interrupts for encoders
    attachPCINT(digitalPinToPCINT(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
    attachPCINT(digitalPinToPCINT(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);

    // Initialize ultrasonic sensors
    ultrasonicSetup(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    ultrasonicSetup(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    ultrasonicSetup(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

    // Initialize MPU6050
    mpuSetup();
}

void MoveForward(int PWM) {
    if (!isMovingForward) {
        isMovingForward = true;
        analogWrite(FNA, PWM);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(FNB, PWM);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        Serial.println("Moving Forward...");
    }
}

void Stop() {
    isMovingForward = false;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Stopped - Left Distance: " + String(leftTotalDistance) + " cm | Right Distance: " + String(rightTotalDistance) + " cm");
}

void TurnLeft() {
    isMovingForward = false;
    analogWrite(FNA, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, 255);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void TurnRight() {
    isMovingForward = false;
    analogWrite(FNA, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(FNB, 255);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void updateDistance() {
    if (isMovingForward) {
        // Calculate distance for each wheel
        float leftDistance = (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        float rightDistance = (rightPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        
        // Update total distance for each wheel
        leftTotalDistance += leftDistance;
        rightTotalDistance += rightDistance;

        // Compute the average distance
        float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;

        // Print distance information
        Serial.print("Left: ");
        Serial.print(leftTotalDistance);
        Serial.print(" cm | Right: ");
        Serial.print(rightTotalDistance);
        Serial.print(" cm | Avg: ");
        Serial.print(avgDistance);
        Serial.println(" cm");

        // Reset pulse counters
        leftPulses = 0;
        rightPulses = 0;

        // Stop when the average distance reaches the target
        if (avgDistance >= TARGET_DISTANCE && !targetReached) {
            Stop();
            targetReached = true;
        }
    }
}

// Return distance in cm
float getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    unsigned long duration = pulseIn(echoPin, HIGH);
    if (duration == 0) {
        Serial.println("Error: No echo received. Check sensor connections or object distance.");
        return 0;  // Return 0 if no echo is received
    }
    return (duration * 0.034613 / 2.00);  // Convert pulse duration to distance in cm
}

void loop() {
    //==========================ULTRASONIC SENSOR=========================================================
    // // Read distances from all three ultrasonic sensors
    // float frontDistance = getDistance(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
    // float leftDistance = getDistance(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    // float rightDistance = getDistance(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

    // // Display ultrasonic sensor readings
    // Serial.print("Front: ");
    // Serial.print(frontDistance);
    // Serial.print(" cm | Left: ");
    // Serial.print(leftDistance);
    // Serial.print(" cm | Right: ");
    // Serial.print(rightDistance);
    // Serial.println(" cm");

     //===========================MOTOR CONTROL + ROTARY ENCODER===========================================

    // // Move forward for 25 cm
    // // if (!targetReached) {
    // //     MoveForward(150);
    // // }

    // MoveForward(150);

    //    // Update and display distance traveled by the wheels
    // updateDistance();

    // // Calculate RPM
    // calculateRPM();

    // // Display data for Serial Plotter (comma-separated values)
    // Serial.print("LeftRaw:");
    // Serial.print(digitalRead(LEFT_ENCODER_PIN));
    // Serial.print(",RightRaw:");
    // Serial.print(digitalRead(RIGHT_ENCODER_PIN));
    // Serial.print(",LeftRPM:");
    // Serial.print(leftRPM);
    // Serial.print(",RightRPM:");
    // Serial.println(rightRPM); // Use println for the last value to end the line

    // // Display data for Serial Monitor (text-based)
    // Serial.print("Left Distance: ");
    // Serial.print(leftTotalDistance);
    // Serial.print(" cm | Right Distance: ");
    // Serial.print(rightTotalDistance);
    // Serial.print(" cm | Avg Distance: ");
    // Serial.print((leftTotalDistance + rightTotalDistance) / 2.0);
    // Serial.println(" cm");

    // // Stop when the target distance is reached
    // if ((leftTotalDistance + rightTotalDistance) / 2.0 >= 25.0 && !targetReached) {
    //     Stop();
    //     targetReached = true;
    // }



    //===============================MPU-6050==============================================================
    // Update MPU6050 angles
    updateMPU();

    // Print MPU6050 angles
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print("° | Pitch: ");
    Serial.print(pitch);
    Serial.print("° | Yaw: ");
    Serial.print(yaw);
    Serial.println("°");

    delay(150);  // Slightly longer delay for smoother readings



}


//=====================================OLD CODE========================================================================
//==================================================================================================================
// #include <Arduino.h>
// #include <PinChangeInterrupt.h>
// #include "NewPing.h"
// #include <limits.h>  // Add this to get INT_MAX

// // Ultrasonic Sensor Pins
// #define FRONT_TRIGGER_PIN A5
// #define FRONT_ECHO_PIN A4
// #define LEFT_TRIGGER_PIN A3
// #define LEFT_ECHO_PIN A2
// #define RIGHT_TRIGGER_PIN A1
// #define RIGHT_ECHO_PIN A0

// // Encoder pin definitions
// #define LEFT_ENCODER_PIN  7
// #define RIGHT_ENCODER_PIN 8

// // Motor Pins
// #define IN1 2
// #define IN2 4
// #define IN3 5
// #define IN4 6
// #define FNA 3
// #define FNB 11

// // Maximum distance for ultrasonic sensors (in centimeters)
// #define MAX_DISTANCE 400

// // Define INT_MAX if it's not already defined (as a fallback)
// #ifndef INT_MAX
// #define INT_MAX 2147483647
// #endif

// // NewPing objects for each ultrasonic sensor
// NewPing front(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
// NewPing left(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
// NewPing right(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);

// // Target distance to travel (in centimeters)
// #define TARGET_DISTANCE 90.0
// #define NODE_DISTANCE_INTERVAL 25  // Distance interval for assigning nodes (in cm)
// #define JUNCTION_DISTANCE_THRESHOLD 15  // Distance threshold to detect a junction (in cm)

// // Define the maximum number of nodes
// #define MAX_NODES 100

// int nodeCounter = 0;  // Counter to keep track of the number of nodes
// int nodeArray[MAX_NODES];   // Array to store node information
// int junctionFlags[MAX_NODES]; // Array to store junction flags (0: no junction, 1: forward, 2: left, 3: right)

// // Encoder and distance calculation variables
// volatile unsigned long leftPulses = 0;
// volatile unsigned long rightPulses = 0;
// const unsigned int PULSES_PER_TURN = 20;
// const float WHEEL_DIAMETER = 4;  // in cm
// const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// // Separate distance tracking for each wheel
// float leftTotalDistance = 0.0;
// float rightTotalDistance = 0.0;
// bool isMovingForward = false;
// bool targetReached = false;

// // Mapping and path-finding variables
// bool visitedNodes[MAX_NODES] = {false};  // Track visited nodes
// bool deadEndNodes[MAX_NODES] = {false};  // Track dead-end nodes
// int path[MAX_NODES];                     // Store the current path
// int pathIndex = 0;                       // Index for the current path
// int shortestPath[MAX_NODES];             // Store the shortest path
// int shortestPathLength = 0;              // Length of the shortest path
// bool endpointDetected = false;           // Flag to detect the endpoint
// bool exploreDone = false;                // Flag to indicate exploration is done
// bool returningToStart = false;           // Flag to indicate returning to start

// // Function declarations
// void leftEncoderISR();
// void rightEncoderISR();
// void MoveForward(int PWM);
// void Stop();
// void TurnRight();
// void TurnLeft();
// void Reverse();
// void updateDistance();
// void Assign_nodes();
// void Explore();
// void Backtrack();
// void CalculateShortestPath();
// void FollowShortestPath();

// // Structure to represent the graph
// struct Graph {
//   int adjacencyMatrix[MAX_NODES][MAX_NODES];
//   int numNodes;
// };

// // Interrupt service routines for encoders
// void leftEncoderISR() {
//     leftPulses++;
// }

// void rightEncoderISR() {
//     rightPulses++;
// }

// void setup() {
//     Serial.begin(9600);
    
//     // Motor Pin Setup
//     pinMode(IN1, OUTPUT);
//     pinMode(IN2, OUTPUT);
//     pinMode(IN3, OUTPUT);
//     pinMode(IN4, OUTPUT);
//     pinMode(FNA, OUTPUT);
//     pinMode(FNB, OUTPUT);

//     // Configure encoder pins
//     pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
//     pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

//     // Initialize interrupts for encoders
//     attachPCINT(digitalPinToPCINT(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
//     attachPCINT(digitalPinToPCINT(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);
    
//     Serial.println("Robot initialized. Starting maze exploration...");
// }

// void MoveForward(int PWM) {
//     if (!isMovingForward) {
//         isMovingForward = true;
//         analogWrite(FNA, PWM);
//         digitalWrite(IN1, LOW);
//         digitalWrite(IN2, HIGH);
//         analogWrite(FNB, PWM);
//         digitalWrite(IN3, HIGH);
//         digitalWrite(IN4, LOW);
//         Serial.println("Moving Forward...");
//     }
// }

// void Stop() {
//     isMovingForward = false;
//     analogWrite(FNA, 0);
//     analogWrite(FNB, 0);
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, LOW);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, LOW);
//     Serial.println("Stopped - Left Distance: " + String(leftTotalDistance) + " cm | Right Distance: " + String(rightTotalDistance) + " cm");
// }

// void TurnRight() {
//     isMovingForward = false;
//     analogWrite(FNA, 255);
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);
//     analogWrite(FNB, 255);
//     digitalWrite(IN3, HIGH);
//     digitalWrite(IN4, LOW);
//     Serial.println("Turning Right...");
//     delay(1000);  // Allow time for the turn to complete
//     Stop();
// }

// void TurnLeft() {
//     isMovingForward = false;
//     analogWrite(FNA, 255);
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, HIGH);
//     analogWrite(FNB, 255);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, HIGH);
//     Serial.println("Turning Left...");
//     delay(1000);  // Allow time for the turn to complete
//     Stop();
// }

// void Reverse() {
//     isMovingForward = false;
//     analogWrite(FNA, 255);
//     digitalWrite(IN1, HIGH);
//     digitalWrite(IN2, LOW);
//     analogWrite(FNB, 255);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, HIGH);
//     Serial.println("Reversing...");
// }

// void updateDistance() {
//     if (isMovingForward) {
//         // Calculate distance for each wheel
//         float leftDistance = (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
//         float rightDistance = (rightPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
        
//         // Update total distance for each wheel
//         leftTotalDistance += leftDistance;
//         rightTotalDistance += rightDistance;

//         // Compute the average distance
//         float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;

//         // Print distance information
//         Serial.print("Left: ");
//         Serial.print(leftTotalDistance);
//         Serial.print(" cm | Right: ");
//         Serial.print(rightTotalDistance);
//         Serial.print(" cm | Avg: ");
//         Serial.print(avgDistance);
//         Serial.println(" cm");

//         // Reset pulse counters
//         leftPulses = 0;
//         rightPulses = 0;

//         // Stop when the average distance reaches the target
//         if (avgDistance >= TARGET_DISTANCE && !targetReached) {
//             Stop();
//             targetReached = true;
//         }
//     }
// }

// void Assign_nodes() {
//     // Calculate the average distance traveled
//     float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;

//     // Check if the robot has traveled the required distance interval
//     if (avgDistance >= (nodeCounter + 1) * NODE_DISTANCE_INTERVAL) {
//         // Assign a node value and store it in the array
//         nodeArray[nodeCounter] = nodeCounter + 1;  // Node value can be an incrementing integer

//         // Read distances from ultrasonic sensors to detect junctions
//         int frontDistance = front.ping_cm();
//         int leftDistance = left.ping_cm();
//         int rightDistance = right.ping_cm();

//         // Flag junctions based on sensor readings
//         if (frontDistance > JUNCTION_DISTANCE_THRESHOLD && 
//             leftDistance > JUNCTION_DISTANCE_THRESHOLD && 
//             rightDistance > JUNCTION_DISTANCE_THRESHOLD) {
//             // T-junction (all three paths available)
//             junctionFlags[nodeCounter] = 7;  // 1 (forward) + 2 (left) + 4 (right) = 7
//         } else if (frontDistance > JUNCTION_DISTANCE_THRESHOLD && 
//                    leftDistance > JUNCTION_DISTANCE_THRESHOLD) {
//             // Forward and left junction
//             junctionFlags[nodeCounter] = 3;  // 1 (forward) + 2 (left) = 3
//         } else if (frontDistance > JUNCTION_DISTANCE_THRESHOLD && 
//                    rightDistance > JUNCTION_DISTANCE_THRESHOLD) {
//             // Forward and right junction
//             junctionFlags[nodeCounter] = 5;  // 1 (forward) + 4 (right) = 5
//         } else if (leftDistance > JUNCTION_DISTANCE_THRESHOLD && 
//                    rightDistance > JUNCTION_DISTANCE_THRESHOLD) {
//             // Left and right junction (T-junction without forward)
//             junctionFlags[nodeCounter] = 6;  // 2 (left) + 4 (right) = 6
//         } else if (frontDistance > JUNCTION_DISTANCE_THRESHOLD) {
//             junctionFlags[nodeCounter] = 1;  // Forward junction
//         } else if (leftDistance > JUNCTION_DISTANCE_THRESHOLD) {
//             junctionFlags[nodeCounter] = 2;  // Left junction
//         } else if (rightDistance > JUNCTION_DISTANCE_THRESHOLD) {
//             junctionFlags[nodeCounter] = 4;  // Right junction
//         } else {
//             junctionFlags[nodeCounter] = 0;  // No junction (dead end)
//         }

//         // Print node and junction information
//         Serial.print("Node ");
//         Serial.print(nodeArray[nodeCounter]);
//         Serial.print(" assigned at ");
//         Serial.print(avgDistance);
//         Serial.print(" cm | Junction Flag: ");
//         Serial.println(junctionFlags[nodeCounter]);

//         // Increment the node counter
//         nodeCounter++;
//     }
// }

// void Explore() {
//     // Check if the current node has been visited
//     if (!visitedNodes[nodeCounter - 1]) {
//         visitedNodes[nodeCounter - 1] = true;  // Mark the node as visited
//         path[pathIndex++] = nodeCounter - 1;   // Add the node to the current path
//     }

//     // Get the junction flag for the current node
//     int junctionFlag = junctionFlags[nodeCounter - 1];
    
//     // Prioritize turning left, then forward, then right
//     if (junctionFlag & 2) {  // Left junction available
//         TurnLeft();
//         delay(500);  // Allow time for the turn to complete
//         MoveForward(150);
//     } else if (junctionFlag & 1) {  // Forward junction available
//         MoveForward(150);
//     } else if (junctionFlag & 4) {  // Right junction available
//         TurnRight();
//         delay(500);  // Allow time for the turn to complete
//         MoveForward(150);
//     } else {
//         // Dead end detected
//         deadEndNodes[nodeCounter - 1] = true;  // Mark the node as a dead end
//         Backtrack();
//     }
// }

// void Backtrack() {
//     // Move back to the previous node
//     if (pathIndex > 0) {
//         int previousNode = path[--pathIndex];  // Remove the current node from the path
//         Serial.print("Backtracking to node ");
//         Serial.println(previousNode);

//         // Reverse direction
//         Reverse();
//         delay(1000);  // Allow time for the rotation to complete
        
//         // Calculate the distance to backtrack
//         float distanceToBacktrack = NODE_DISTANCE_INTERVAL;
        
//         // Reset distance counters for backtracking
//         leftPulses = 0;
//         rightPulses = 0;
        
//         // Move backward until reaching the previous node
//         while ((leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE < distanceToBacktrack) {
//             // Keep moving backward
//             delay(100);
//         }
        
//         Stop();
        
//         // Determine which way to go from the previous node
//         // This would need to be based on your maze structure and junction flags
//         // For now, we'll just try to move in a direction that's not a dead end
//         int junctionFlag = junctionFlags[previousNode];
        
//         if (junctionFlag & 1 && !(deadEndNodes[previousNode + 1])) {  // Forward path available and not dead end
//             MoveForward(150);
//         } else if (junctionFlag & 2 && !(deadEndNodes[previousNode - 1])) {  // Left path available and not dead end
//             TurnLeft();
//             delay(500);
//             MoveForward(150);
//         } else if (junctionFlag & 4 && !(deadEndNodes[previousNode + 1])) {  // Right path available and not dead end
//             TurnRight();
//             delay(500);
//             MoveForward(150);
//         } else {
//             // All paths from this node lead to dead ends
//             deadEndNodes[previousNode] = true;  // Mark this node as a dead end too
//             Backtrack();  // Backtrack further
//         }
//     } else {
//         // We've backtracked to the starting point and all paths have been explored
//         Serial.println("Exploration complete. Calculating shortest path...");
//         exploreDone = true;
//         CalculateShortestPath();
//     }
// }

// // Initialize the graph
// void initGraph(struct Graph* graph, int numNodes) {
//   graph->numNodes = numNodes;
  
//   // Initialize adjacency matrix with 0 (no connections)
//   for (int i = 0; i < numNodes; i++) {
//     for (int j = 0; j < numNodes; j++) {
//       graph->adjacencyMatrix[i][j] = 0;
//     }
//   }
// }

// // Add an edge to the graph
// void addEdge(struct Graph* graph, int src, int dest, int weight) {
//   graph->adjacencyMatrix[src][dest] = weight;
//   // For an undirected graph, uncomment the line below
//   graph->adjacencyMatrix[dest][src] = weight;  // Make it undirected since we can travel both ways
// }

// // Find the vertex with the minimum distance value
// int minDistance(int dist[], bool sptSet[], int numNodes) {
//   int min = INT_MAX, min_index = -1;
  
//   for (int v = 0; v < numNodes; v++) {
//     if (!sptSet[v] && dist[v] <= min) {
//       min = dist[v];
//       min_index = v;
//     }
//   }
  
//   return min_index;
// }

// // Print the constructed path
// void printPath(int parent[], int j) {
//   // Base case: if j is the source
//   if (parent[j] == -1)
//     return;
  
//   printPath(parent, parent[j]);
//   Serial.print(j);
//   Serial.print(" ");
// }

// // Store the path in the shortestPath array
// void storePath(int parent[], int j, int shortestPath[], int& pathIndex) {
//   // Base case: if j is the source
//   if (parent[j] == -1)
//     return;
  
//   storePath(parent, parent[j], shortestPath, pathIndex);
//   shortestPath[pathIndex++] = j;
// }

// // Implement Dijkstra's algorithm
// void dijkstra(struct Graph* graph, int src, int dest, int shortestPath[], int& shortestPathLength) {
//   int numNodes = graph->numNodes;
//   int dist[MAX_NODES];     // The output array dist[i] holds the shortest distance from src to i
//   bool sptSet[MAX_NODES];  // sptSet[i] will be true if vertex i is included in the shortest path tree
//   int parent[MAX_NODES];   // Parent array to store the shortest path tree
  
//   // Initialize all distances as INFINITE and sptSet[] as false
//   for (int i = 0; i < numNodes; i++) {
//     dist[i] = INT_MAX;
//     sptSet[i] = false;
//     parent[i] = -1;  // No parent yet
//   }
  
//   // Distance of source vertex from itself is always 0
//   dist[src] = 0;
  
//   // Find shortest path for all vertices
//   for (int count = 0; count < numNodes - 1; count++) {
//     // Pick the minimum distance vertex from the set of vertices not yet processed
//     int u = minDistance(dist, sptSet, numNodes);
    
//     // Mark the picked vertex as processed
//     sptSet[u] = true;
    
//     // If we've reached our destination, we can break early
//     if (u == dest) {
//       break;
//     }
    
//     // Update dist value of the adjacent vertices of the picked vertex
//     for (int v = 0; v < numNodes; v++) {
//       // Update dist[v] only if:
//       // 1. There is an edge from u to v
//       // 2. The vertex is not in the shortest path tree
//       // 3. The distance through u is smaller than the current value of dist[v]
//       if (!sptSet[v] && graph->adjacencyMatrix[u][v] && 
//           dist[u] != INT_MAX && dist[u] + graph->adjacencyMatrix[u][v] < dist[v]) {
//         dist[v] = dist[u] + graph->adjacencyMatrix[u][v];
//         parent[v] = u;
//       }
//     }
//   }
  
//   // Print the constructed distance array
//   Serial.println("Vertex \t Distance from Source");
//   for (int i = 0; i < numNodes; i++) {
//     Serial.print(i);
//     Serial.print(" \t\t ");
//     Serial.println(dist[i]);
//   }
  
//   // Print and store the path to destination
//   Serial.print("Shortest path from ");
//   Serial.print(src);
//   Serial.print(" to ");
//   Serial.print(dest);
//   Serial.print(": ");
  
//   int pathIndex = 0;
//   shortestPath[pathIndex++] = src;  // Add source node to the path
  
//   storePath(parent, dest, shortestPath, pathIndex);
//   shortestPathLength = pathIndex;
  
//   // Print the path
//   Serial.print(src);
//   Serial.print(" ");
//   printPath(parent, dest);
//   Serial.println();
  
//   Serial.print("Total distance: ");
//   Serial.println(dist[dest]);
// }

// void CalculateShortestPath() {
//   Serial.println("Calculating shortest path using Dijkstra's algorithm...");
  
//   // Create a graph based on the nodes and junctions we've discovered
//   struct Graph graph;
//   initGraph(&graph, nodeCounter);
  
//   // Create edges based on the junctions
//   for (int i = 0; i < nodeCounter; i++) {
//     // Extract the junction flag
//     int jFlag = junctionFlags[i];
    
//     // Add edges based on the junction flags
//     if (jFlag & 1) {  // Forward junction
//       if (i + 1 < nodeCounter) {  // Make sure we don't go out of bounds
//         addEdge(&graph, i, i + 1, 1);  // Weight = 1 for simplicity
//       }
//     }
    
//     if (jFlag & 2) {  // Left junction
//       // In a grid-based maze, left would typically be the node to the left
//       // This is a simplification and may need adjustment based on your maze structure
//       if (i > 0) {  // Make sure we don't go out of bounds
//         addEdge(&graph, i, i - 1, 1);
//       }
//     }
    
//     if (jFlag & 4) {  // Right junction
//       // In a grid-based maze, right would typically be the node to the right
//       if (i + 1 < nodeCounter) {  // Make sure we don't go out of bounds
//         addEdge(&graph, i, i + 1, 1);
//       }
//     }
//   }
  
//   // Run Dijkstra's algorithm from the start node (0) to the end node (nodeCounter - 1)
//   dijkstra(&graph, 0, nodeCounter - 1, shortestPath, shortestPathLength);
  
//   // Print the shortest path
//   Serial.println("Final shortest path:");
//   for (int i = 0; i < shortestPathLength; i++) {
//     Serial.print(shortestPath[i]);
//     if (i < shortestPathLength - 1) {
//       Serial.print(" -> ");
//     }
//   }
//   Serial.println();
  
//   // Set the flag to start following the shortest path
//   returningToStart = true;
//   exploreDone = true;
// }

// void FollowShortestPath() {
//   static int currentPathIndex = 0;
  
//   if (currentPathIndex < shortestPathLength) {
//     int targetNode = shortestPath[currentPathIndex];
    
//     // Calculate the average distance traveled
//     float avgDistance = (leftTotalDistance + rightTotalDistance) / 2.0;
    
//     // Check if we've reached the target node
//     if (abs(avgDistance - targetNode * NODE_DISTANCE_INTERVAL) < 5) {  // 5 cm tolerance
//       Serial.print("Reached node ");
//       Serial.println(targetNode);
//       currentPathIndex++;
      
//       if (currentPathIndex < shortestPathLength) {
//         // Determine which direction to move next
//         int nextNode = shortestPath[currentPathIndex];
        
//         if (nextNode > targetNode) {
//           // Move forward
//           MoveForward(150);
//         } else if (nextNode < targetNode) {
//           // Move backward
//           Reverse();
//           delay(500);
//           MoveForward(150);
//         }
//       } else {
//         // We've reached the end of the path
//         Stop();
//         Serial.println("Reached the endpoint via the shortest path!");
//       }
//     }
//   }
// }

// void loop() {
//     // Read distance from ultrasonic sensors
//     int frontDistance = front.ping_cm();
//     int leftDistance = left.ping_cm();
//     int rightDistance = right.ping_cm();

//     // Update and display distance only while moving
//     updateDistance();

//     // If we're still exploring, assign nodes and detect junctions
//     if (!exploreDone) {
//         Assign_nodes();
//     }

//     // Check if the endpoint is detected (all sensors > 25 cm)
//     if (!endpointDetected && frontDistance > 25 && leftDistance > 25 && rightDistance > 25) {
//         endpointDetected = true;
//         Stop();
//         Serial.println("Endpoint detected at node " + String(nodeCounter - 1) + "!");
//         CalculateShortestPath();  // Calculate the shortest path using Dijkstra's algorithm
//     }

//     // Main state machine for the robot's behavior
//     if (returningToStart) {
//         // Following the shortest path back to start
//         FollowShortestPath();
//     } else if (endpointDetected) {
//         // We've found the endpoint, wait for instructions
//         if (!exploreDone) {
//             CalculateShortestPath();
//         }
//     } else {
//         // Exploring mode
//         if (deadEndNodes[nodeCounter - 1]) {
//             // Current node is a dead end, backtrack
//             Backtrack();
//         } else {
//             // Move forward and prioritize turning left
//             Explore();
//         }
//     }

//     delay(150);  // Slightly longer delay for smoother readings
// }