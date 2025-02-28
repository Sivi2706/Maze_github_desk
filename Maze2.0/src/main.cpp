#include <Arduino.h>
#include <PinChangeInterrupt.h>
#include <MPU6050_tockn>

/*
Improvements: 
1. Implementing MPU to turn the car rather than using delay
2. Stuck detection - When rotary encoder is increasing but no change to ultrasonic distance, the car is stuck
*/

// Ultrasonic Sensor Pins

#define FRONT_TRIGGER_PIN A5
#define FRONT_ECHO_PIN A4
#define LEFT_TRIGGER_PIN A3
#define LEFT_ECHO_PIN A2
#define RIGHT_TRIGGER_PIN A1
#define RIGHT_ECHO_PIN A0


// Encoder pin definitions
#define LEFT_ENCODER_PIN 7
#define RIGHT_ENCODER_PIN 8

// Motor Pins
#define IN1 2
#define IN2 4
#define IN3 5
#define IN4 6
#define FNA 3
#define FNB 11

// Maximum distance for ultrasonic sensors (in centimeters)
#define MAX_DISTANCE 400

//IMPORTANT VARIABLES TO BE DEFINED
int initial_point[2] = {0, 0};

// Encoder and distance calculation variables
volatile unsigned long leftPulses = 0;
volatile unsigned long rightPulses = 0;
const unsigned int PULSES_PER_TURN = 20;
const float WHEEL_DIAMETER = 4;  // in cm
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Separate distance tracking for each wheel
float leftTotalDistance = 0.0;
float rightTotalDistance = 0.0; //could add a more accurate total distance combining ultrasonic
float initialUltrasonic = 0.0;
float accurateDistance = 0.0; //including both rotary encoder and ultrasonic
bool isMoving = false;
int direction = 2; // 1 = left, 2 = front, 3 = right, 4 = 180 reverse
bool targetReached = false;

// Check if car is stuck
float ultrasonicDistance = 0;

// Check the surroundings of the car
bool leftOpening = false;
bool frontOpening = false;
bool rightOpening = false; 

void leftEncoderISR();
void rightEncoderISR();
void forward(int PWM);
void stop();
void left();
void right();
void reverse();
void updateDistance();
float ultrasonicPulse(int trigPin, int echoPin);
void initialUltrasonicDistance();
void checkUltrasonicDistance();

struct Position {
    int x;
    int y;
}; //coordinates of each cell in the maze

void setup() {
    Serial.begin(115200);

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
    attachInterrupt(digitalPinToPCINT(LEFT_ENCODER_PIN), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToPCINT(RIGHT_ENCODER_PIN), rightEncoderISR, CHANGE);
}

// Interrupt service routines for encoders
void leftEncoderISR() {
    leftPulses++;
}

void rightEncoderISR() {
    rightPulses++;
}

void forward(int PWM) {
    direction = 2;
    isMoving = true;
    analogWrite(FNA, PWM);
    analogWrite(FNB, PWM);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving Forwards");
}

void right() {
    direction = 3;
    isMoving = true;
    analogWrite(FNA, 255);
    analogWrite(FNB, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    Serial.println("Moving right");
}

void left() {
    direction = 1;
    isMoving = true;
    analogWrite(FNA, 255);
    analogWrite(FNB, 255);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Moving left");
}

void reverse() { //instead of reversing, maybe should turn 180 and just move forwards 
    /*Problem is when car turns 180, when it starts moving forwards direction = 1 and the distance would be added again
    */
    direction = 4;
    isMoving = true;
    analogWrite(FNA, 255);
    analogWrite(FNB, 255);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    Serial.println("Reversing"); 
}

void stop() {
    isMoving = false;
    analogWrite(FNA, 0);
    analogWrite(FNB, 0);
    Serial.println("Stop");
}

void updateDistance() {
    float leftDistance = (leftPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;
    float rightDistance = (rightPulses / (float)PULSES_PER_TURN) * WHEEL_CIRCUMFERENCE;

    //check the direction that the car is moving and then decide if add or minus distance
    //use the distance to determine if the car has moved up a 'block'
    //can also use ultrasonic and distance together to make it more accurate

    float avgDistance = (leftTotalDistance + rightTotalDistance) / 2;
    
    if (direction == 4) {
        leftTotalDistance -= avgDistance;
        rightTotalDistance -= avgDistance;
    } else {
        leftTotalDistance += avgDistance;
        rightTotalDistance += avgDistance;
    }

    leftPulses = 0;
    rightPulses = 0;
}

void checkMPU() {
    //stuff
}

float ultrasonicPulse(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delay(5);
    digitalWrite(trigPin, HIGH);
    delay(10);
    digitalWrite(trigPin, LOW);

    long time = pulseIn(echoPin, HIGH);
    return time/2*0.343;
}

void initialUltrasonicDistance() {
    initialUltrasonic = ultrasonicPulse(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
}

void checkUltrasonicDistance() {
    float finalUltrasonic = ultrasonicPulse(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);

    float distanceDelta = initialUltrasonic - finalUltrasonic;

    accurateDistance = (leftTotalDistance + rightTotalDistance + distanceDelta)/3;

    initialUltrasonic = 0;
}

void checkSurroundings() {
    float frontDistance = ultrasonicPulse(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN); //this only returns time duration
    float leftDistance = ultrasonicPulse(LEFT_TRIGGER_PIN, LEFT_ECHO_PIN);
    float rightDistance = ultrasonicPulse(RIGHT_TRIGGER_PIN, RIGHT_ECHO_PIN);

    if (leftDistance > 25) leftOpening = true;
<<<<<<< Updated upstream
    if (frontDistance > 25) frontOpening = true;
    if (rightDistance > 25) rightOpening = true;
=======
    if (rightDistance > 25) rightOpening = true;
    if (frontDistance > 25) frontOpening = true;
>>>>>>> Stashed changes
}

bool checkStuck() {
    if (isMoving) {
        float distanceNow = ultrasonicPulse(FRONT_TRIGGER_PIN, FRONT_ECHO_PIN);
        float distanceDelta = ultrasonicDistance - distanceNow;

        ultrasonicDistance = distanceNow;
        
        if (distanceDelta > 0.1 || distanceDelta < -0.1) return false;
        return true
    }
}

void explore() {
    int maze[8][8] = {};

    Position current = {0, 0};
<<<<<<< Updated upstream

    Position explored[64] = {current};

    while (1) {
        checkSurroundings(); //check if there are openings around it
    }
}

void djisktra() {
    int maze[10][10] = {}; //initializes an empty 2D array that is used to represent the maze

    directions = [{1, 0}, {-1, 0}, {0, 1}, {0, -1}]; //right, left, up, down

    

}



void loop() {
    initialUltrasonicDistance(); //set initial ultrasonic distance
    if (checkStuck()) {
        reverse(); //if the car is stuck, reverse
    }
    //...do stuff
    checkUltrasonicDistance();
}

/*
{{0, 0, 0, 0, 0}, 
 {0, 0, 0, 0, 0},
 {0, 0, 0, 0, 0},
 {0, 0, 0, 0, 0},
 {0, 0, 0, 0, 0}
 }


=======

    Position explored[64] = {current};


}

void djisktra() {
    int maze[10][10] = {}; //initializes an empty 2D array that is used to represent the maze

    directions = [{1, 0}, {-1, 0}, {0, 1}, {0, -1}]; //right, left, up, down

    

}



void loop() {
    initialUltrasonicDistance(); //set initial ultrasonic distance
    if (checkStuck()) {
        reverse(); //if the car is stuck, reverse
    }
    //...do stuff
    checkUltrasonicDistance();
}

/*
{{0, 0, 0, 0, 0}, 
 {0, 0, 0, 0, 0},
 {0, 0, 0, 0, 0},
 {0, 0, 0, 0, 0},
 {0, 0, 0, 0, 0}
 }


>>>>>>> Stashed changes
*/