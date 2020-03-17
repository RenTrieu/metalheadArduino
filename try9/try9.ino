/*
 * Program: metalhead.ino
 * Author: Darren Trieu Nguyen
 * Last Modified: 3-16-2020
 * Function: Operates the metalhead sensors and policy
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <QMC5883L.h>
#include <Wire.h>

/*
 * Coordinate Node: Stores coordinate information and pointers to
 *                  the next and previous nodes using a doubly linked list
 *                  format
 */
typedef struct CoordNode {
    int nodeNumber;         // The "index" of the CoordNode
    double distance;
    int angle;
    CoordNode* prev;
    CoordNode* next;
} CoordNode;

/* Compass (QMC5883L) Declaration */
QMC5883L compass;

/* Setting the motor driver pins that control the treads */

// Right Motor Pins
int inp1R = 5;              // Motor Pins
int inp2R = 6;
int enb = 7;                // PWM Speed Driver Pin

// Left Motor Pins
int inp1L = 8;              // Motor Pins
int inp2L = 9;
int ena = 10;               // PWM Speed Driver Pin

/* Setting the IR sensor pins */
int centerIR = 18;
int leftFrontIR = 2;
int rightFrontIR = 3;
int leftBackIR = 50;
int rightBackIR = 51;

/* Speed of the treads */
int treadSpeed = 80;

/* Coordinate Node Declaration */
CoordNode* root;
CoordNode* curNode;

/* Behavior/Calibration Constants */

double cmPerSecond = (15.0/70.0)*treadSpeed;
// This is the estimated forward/backward tread speed of the metalhead chassis 
// DC motors. This is used to convert the amount of time the metalhead spends
// traveling forward/backward into distance in centimeters

double turnSpeedBase = treadSpeed + 10.0;
// This value tells the metalhead how fast to move its treads when turning

double timeIncrement = 400;
// This is a constant timeIncrement that is used to space out drive function 
// calls

/* Behavior/Calibration Variables */
double reverseTimeOffset = 100; // This variable is used to account for
                                // the difference in recorded distance/time
                                // created during forward motion in the
                                // retraverse function (due to checking to see
                                // if the IR sensors see anything)

int angleTolerance = 5;         // Determines the +/- range of which angles are
                                // acceptable for angAlign

unsigned long curTime;          // Variable to hold current/immediate time
unsigned long prevTime;         // Variable to hold the time from a previous
                                // iteration of a loop

String curMode;                 // Holds the current mode for the
                                // drive function

String curTurn;                 // Holds the current direction 
                                // (clock/counterclock) for turning

int curAngle;                   // Holds the current angle

int stopTime = 500;             // Delay after stopping for letting motors rest
                                // in milliseconds

int maxNodes = 3;               // Constant to determine the max number of
                                // nodes the metalhead will create before
                                // reversing to home and entering Phase 1

int phase = 0;                  // Denotes the phase of the metalhead
                                // Phase 0: Mapping out a path
                                //          When finished, reverse to home
                                // Phase 1: Retraverse the mapped path, if see
                                //          obstacle, teabag and then continue
                                // Phase 2: Stop

void setup() {
    /* Initializating */
    Serial.begin(9600);

    /* Initializing Tread Driving Pins */
    pinMode(inp1R, OUTPUT);
    pinMode(inp2R, OUTPUT);
    pinMode(inp1L, OUTPUT);
    pinMode(inp2L, OUTPUT);
    pinMode(ena, OUTPUT);
    pinMode(enb, OUTPUT);

    /* Initializing IR Proximity Sensor Pins */
    pinMode(leftFrontIR, INPUT);
    pinMode(leftBackIR, INPUT);
    pinMode(rightFrontIR, INPUT);
    pinMode(rightBackIR, INPUT);
    pinMode(centerIR, INPUT);

    /* Initializing wire and compass */
    Wire.begin();
    compass.init();
    compass.setSamplingRate(50);

    /* Calibrating compass */
    Serial.println("Calibrating");
    drive("clock", turnSpeedBase - 10);
    delay(4000);
    drive("stop", 0);
    drive("counterclock", turnSpeedBase - 10);
    delay(4000);
    drive("stop", 0);
    delay(2000);
    Serial.println("Calibrating Complete");
    
    /* Initializing Root Node */
    root = (CoordNode*) malloc(sizeof(CoordNode));
    root->nodeNumber = 0;
    root->distance = 0.0;
    root->angle = 0.0;
    root->prev = NULL;
    root->next = NULL;
    curNode = root;

    /* Initializing random number generator with noise read from pin 0 */
    srand(analogRead(0));

    /* Initializing Behavior */
    curMode = "forward";
    curTurn = "clock";
    curAngle = compass.readHeading();
    curTime = millis();
    prevTime = curTime;
}

void loop() {
    /* Phase 0 */
    if (phase == 0) {
        double distTraveled = 0.0;
        drive(curMode.c_str(), treadSpeed);

        /* Trigger Node Creation if obstacle is detected close to metalhead */
        if ((digitalRead(leftFrontIR) == 0) 
            || (digitalRead(rightFrontIR) == 0)
            || (digitalRead(centerIR) == 0)) {
            /* Halting movement if an object is close by */
            curTime = millis();
            drive("stop", 0);
            delay(stopTime);
            Serial.println("Stopping");
            double secondsElapsed = (double) (curTime - prevTime) / 1000.0;
            double curDistance = secondsElapsed * cmPerSecond;

            /* Go backwards by some increment */
            Serial.println("Backing up");
            drive("backward", treadSpeed);
            delay(timeIncrement);
            drive("stop", 0);
            delay(stopTime);
            curDistance -= (timeIncrement / 1000.0) * cmPerSecond;

            /* calculating the distance relative to the previous coordnode */
            delay(1000);
            curAngle = compass.readHeading();
            Serial.println("Creating Node");
            Serial.print("Making node with absolute angle: ");
            Serial.println(curAngle);
            createNode(curDistance, curAngle);

            /* Behavior after detecting obstacle */
            if (curNode->nodeNumber == maxNodes) {
                Serial.println("REVERSE");
                reverse(root);
                phase = 1;
            }
            else {
                double timeTurning = 0.0;
                /* Keep trying random angles until the path is clear */
                int randTime = (int) generateRandomDouble(1000, 1500, 0);
                int randDirection = (int) generateRandomDouble(-1, 1, 0);
                if (randDirection >= 0) {
                    drive("clock", treadSpeed);
                    delay(randTime);
                    drive("stop", 0);
                    delay(stopTime);
                }                
                else {
                    drive("counterclock", treadSpeed);
                    delay(randTime);
                    drive("stop", 0);
                    delay(stopTime);
                }
             
                curAngle = compass.readHeading();
                curMode = "forward";
                curAngle = curAngle % 360;
                prevTime = millis();
            }
        }
    }
    /* Phase 1 */
    else if (phase == 1) {
        retraverse(root);
        drive("stop", 0);
        delay(stopTime);
        phase = 2;
    }
    /* Phase 2 */
    else {
        reverse(root);
        drive("stop", 0);
        delay(stopTime);
        retraverse(root);
        drive("stop", 0);
        delay(stopTime);
    }
    delay(10);
}

/*
 * Create Node Function: Creates a new node and stores the current relative
 *                       distance and angle (relative to previous node)
 */
void createNode(double relDist, double relAngle) {
    /* Allocating memory on the heap for the new node */
    CoordNode* newNode = (CoordNode*) malloc(sizeof(CoordNode));

    /* Pointing the current node and new node to each other */
    curNode->next = newNode;
    newNode->prev = curNode;

    /* Storing the corresponding initial data in newNode */
    newNode->nodeNumber = curNode->nodeNumber + 1;
    newNode->distance = relDist;
    newNode->angle = relAngle;
    newNode->next = NULL;

    /* Updating the curNode pointer */
    curNode = newNode;
}

/*
 * Drive function: Manages the driving state of the metalhead
 */
void drive(String modeIn, int driveSpeed) {

    /* Constants accounting for differences in the DC motors/tread speed */
    int leftForwardOffset = -3;
    int rightForwardOffset = 0;
    int leftBackOffset = -3;
    int rightBackOffset = 0;
    int leftOffset = -3;
    int rightOffset = 0;

    const char * mode = modeIn.c_str();
    if (strcmp(mode, "forward") == 0) {
      /* sets right tread to move forward */
        digitalWrite(inp1R, HIGH);
        digitalWrite(inp2R, LOW);
      /* sets left tread to move forward */
        digitalWrite(inp1L, HIGH);
        digitalWrite(inp2L, LOW);

      /* sets speed of the tank */  
        analogWrite(ena, driveSpeed + leftForwardOffset);
        analogWrite(enb, driveSpeed + rightForwardOffset);
    }
    else if (strcmp(mode, "backward") == 0) {
      /* sets right and left tread to move backward */  
        digitalWrite(inp1R, LOW);
        digitalWrite(inp2R, HIGH);
        digitalWrite(inp1L, LOW);
        digitalWrite(inp2L, HIGH);
        
        analogWrite(ena, driveSpeed + leftBackOffset);
        analogWrite(enb, driveSpeed + rightBackOffset);
    }
    else if (strcmp(mode, "clock") == 0) {
      /* set right tread to move forward */
        digitalWrite(inp1R, HIGH);
        digitalWrite(inp2R, LOW);

      /* set left tread to move backward */
        digitalWrite(inp1L, LOW);
        digitalWrite(inp2L, HIGH);

        analogWrite(ena, driveSpeed + leftOffset);
        analogWrite(enb, driveSpeed + rightOffset);
    }
    else if (strcmp(mode, "counterclock") == 0) {
      /* set right tread to move backward */
        digitalWrite(inp1R, LOW);
        digitalWrite(inp2R, HIGH);

      /* set left tread to move forward */
        digitalWrite(inp1L, HIGH);
        digitalWrite(inp2L, LOW);

        analogWrite(ena, driveSpeed + leftOffset);
        analogWrite(enb, driveSpeed + rightOffset); 
    }
    else if (strcmp(mode, "stop") == 0) {
        digitalWrite(inp1R, LOW);
        digitalWrite(inp2R, LOW);
        digitalWrite(inp1L, LOW);
        digitalWrite(inp2L, LOW);
    }
    else {
        digitalWrite(inp1R, LOW);
        digitalWrite(inp2R, LOW);
        digitalWrite(inp1L, LOW);
        digitalWrite(inp2L, LOW);
    }
}

/*
 * Generates a random double between two integer ranges
 */
double generateRandomDouble(int minValue, int maxValue, int precision) {
    double randDouble;
    int randInt;
    /* Non-Zero Precision Case */
    if (precision > 0) {
        /* Generating a random integer through standard rand() means */
        randInt = (rand() % ((maxValue - minValue)*10*precision))
                             + minValue*10*precision;
        /* Converting randInt to double of the corresponding precision */
        randDouble = ((double) randInt) / (10.0* (double) precision);

    }
    /* Zero Precision Case */
    else {
        /* Generating a random integer through standard rand() means */
        randInt = (rand() % (maxValue - minValue)) + minValue;
        /* Converting randInt to double of the corresponding precision */
        randDouble = (double) randInt;
    }
    return randDouble;
}

/* 
 * Returns the absolute cartesian coordinates of a CoordNode
 */
double* absCartesian(CoordNode* cNode) {
    /* 0 is x and 1 is y */
    double* cartesianCoords = (double *) malloc(2*sizeof(double));
    cartesianCoords[0] = cNode->distance * cos(cNode->angle * M_PI / 180.0);
    cartesianCoords[1] = cNode->distance * sin(cNode->angle * M_PI / 180.0);
    if (cNode->prev != NULL) {
        double* prevCoords = absCartesian(cNode->prev);
        cartesianCoords[0] += prevCoords[0];
        cartesianCoords[1] += prevCoords[1];
        free(prevCoords);
        prevCoords = NULL;
    }
    return cartesianCoords;
}

/* 
 * Tells the metalhead to reverse it's course and to the root CoordNode
 * given the root CoordNode
 */
void reverse(CoordNode* cNode) {
    /* Base Case: Current Node */
    if (cNode->next == NULL) {
        /* Aligning to the node's angle */
        angAlign((cNode->angle) % 360, 0);
        Serial.println("Base Case");
        Serial.print("Align to ");
        Serial.println((cNode->angle) % 360);

        /* Driving node's distance backwards */
        double driveTime = (fabs(cNode->distance) / cmPerSecond) * 1000.0;
        drive("backward", treadSpeed);
        delay(driveTime - reverseTimeOffset);
        drive("stop", treadSpeed);
        delay(stopTime);
        return;
    }
    /* Other non-root Nodes  */
    else if (cNode->prev != NULL) {
        /* Calling the next node */
        reverse(cNode->next);

        /* Aligning */
        angAlign((cNode->angle) % 360, 0);
        Serial.println("Non - Base Case");
        Serial.print("Align to ");
        Serial.println((cNode->angle) % 360);

        /* Driving node's distance */
        double driveTime = (fabs(cNode->distance) / cmPerSecond) * 1000.0;
        drive("backward", treadSpeed);
        delay(driveTime - reverseTimeOffset);
        drive("stop", treadSpeed);
        delay(stopTime);
        return;
    }
    /* Root Case */
    else {
        reverse(cNode->next);
        return;
    }

}

/*
 * Function: angAlign - Aligns the metalhead with a given absolute angle 
 * Parameters: int angle - Absolute angle with which to align
 *             int direction - The direction to turn to:
 *                             -1 is counterclockwise
 *                             0 if direction is the shortest path to angle
 *                             1 is clockwise
 * Returns: N/A
 * Side Effects: Aligns the metalhead with the given angle and sets
 *               curAngle (global) to this angle
 */ 
void angAlign(int angle, int direction) {
    /* Shortest Path Case */
    if (direction == 0) {
        int angDifference1 = (angle - curAngle) % 360;
        int angDifference2 = (curAngle - angle) % 360;
        if (curAngle >= 180) {
            /* Clockwise Shortest Path Case */
            if (angDifference1 > angDifference2) {
                if (curMode != "clock") {
                    drive("clock", turnSpeedBase);
                    while ((curAngle <= (angle - angleTolerance))
                           || (curAngle >= (angle + angleTolerance))) {
                        curAngle = compass.readHeading();
                    }
                    drive("stop", 0);
                    delay(stopTime);
                }
            }
            /* Counterclockwise Shortest Path Case */
            else {
                if (curMode != "counterclock") {
                    drive("counterclock", turnSpeedBase);
                    while ((curAngle <= (angle - angleTolerance))
                           || (curAngle >= (angle + angleTolerance))) {
                        curAngle = compass.readHeading();
                    }
                    drive("stop", 0);
                    delay(stopTime);
                }
            }
        }
        else {
            /* Clockwise Shortest Path Case */
            if (angDifference1 < angDifference2) {
                if (curMode != "clock") {
                    drive("clock", turnSpeedBase);
                    while ((curAngle <= (angle - angleTolerance))
                           || (curAngle >= (angle + angleTolerance))) {
                        curAngle = compass.readHeading();
                    }
                    drive("stop", 0);
                    delay(stopTime);
                }
            }
            /* Counterclockwise Shortest Path Case */
            else {
                if (curMode != "counterclock") {
                    drive("counterclock", turnSpeedBase);
                    while ((curAngle <= (angle - angleTolerance))
                           || (curAngle >= (angle + angleTolerance))) {
                        curAngle = compass.readHeading();
                    }
                    drive("stop", 0);
                    delay(stopTime);
                }
            }

        }
    }
    /* Clockwise Case */
    else if (direction == 1) {
        if (curMode != "clock") {
            drive("clock", turnSpeedBase);
            while ((curAngle <= (angle - angleTolerance))
                   || (curAngle >= (angle + angleTolerance))) {
                curAngle = compass.readHeading();
            }
            drive("stop", 0);
            delay(stopTime);
        }
    }
    /* Counterclockwise Case */
    else if (direction == -1) {
        if (curMode != "counterclock") {
            drive("counterclock", turnSpeedBase);
            while ((curAngle <= (angle - angleTolerance))
                   || (curAngle >= (angle + angleTolerance))) {
                curAngle = compass.readHeading();
            }
            drive("stop", 0);
            delay(stopTime);
        }
    }
    curAngle = compass.readHeading();
    drive("stop", 0);
    curAngle = compass.readHeading();
    curMode = "forward";

}

/* 
 * Retraverse Function: Re-traverse the CoordNode list mapping
 */
void retraverse(CoordNode* cNode) {
    /* Aligning to the node's angle */
    angAlign((cNode->angle) % 360, 0);
    Serial.println("Base Case");
    Serial.print("Align to ");
    Serial.println((cNode->angle) % 360);

    /* Driving node's distance */
    double driveTime = (fabs(cNode->distance) / cmPerSecond) * 1000.0;
    double timeIncrement = 0;

    drive("forward", treadSpeed);
    while (timeIncrement <= (driveTime)) {
        /* If the metalhead detects something, it will wiggle and then
           try to run it over, wiggle again,
           then it will continue along its path */
        if ((digitalRead(leftFrontIR) == 0) 
            || (digitalRead(rightFrontIR) == 0)
            || (digitalRead(centerIR) == 0)) {
            drive("stop", 0);
            delay(1000);
            drive("counterclock", treadSpeed);
            delay(100);
            drive("clock", treadSpeed);
            delay(100);
            drive("stop", 0);
            delay(500);
            drive("forward", treadSpeed);
            delay(500);
            drive("stop", 0);
            delay(stopTime);
            drive("counterclock", treadSpeed);
            delay(100);
            drive("clock", treadSpeed);
            delay(100);
            drive("stop", 0);
            delay(stopTime);
            drive("backward", treadSpeed);
            delay(500);
            drive("forward", treadSpeed);
        }
        else {
            timeIncrement += 1;
            delay(1);
        }
    }

    drive("stop", treadSpeed);
    delay(stopTime);

    /* Base Case: Final Node */
    if (cNode->next == NULL) {
        return;
    }
    /* Recursive Case: Any Node that is not the final node */
    else {
        /* Calling the next node */
        retraverse(cNode->next);
        return;
    }
}
