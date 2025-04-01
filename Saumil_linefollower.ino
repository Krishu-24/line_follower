#include <AFMotor.h>

// Initialize motors on correct ports
AF_DCMotor motorLeft(2);   // Motor connected to M2
AF_DCMotor motorRight(1);  // Motor connected to M1

// Sensor configuration
#define NUM_SENSORS 7
const int C = A0;  
const int R3 = A1;  
const int L1 = A2;  
const int R2 = A3;   
const int L3 = A4;  
const int L2 = A5;  
const int R1 = 2;

const int sensorPins[NUM_SENSORS] = {L3, L2, L1, C, R1, R2, R3};
int sensorValues[NUM_SENSORS];

// PID Constants (tune these as needed)
float Kp = 0.9;     
float Ki = 0.001;   
float Kd = 3.5;     

// PID Variables
int error, lastError = 0;
int P, I, D, PIDvalue;
int baseSpeed = 200;     // Full power
float speedFactor = 0.45; // Scaling factor for motor speed

// Speed control during turns
float turnSpeedMultiplier = 0.5;  // Slow down for sharper turns

void setup() {
    Serial.begin(115200);
    
    // Initialize motors and start them
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);
    motorLeft.run(FORWARD);
    motorRight.run(FORWARD);

    // Set sensor pins as input
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
}

void loop() {
    int position = readSensors();
    error = (NUM_SENSORS - 1) * 500 - position;
    
    // PID calculations
    P = error;
    I += error;
    D = error - lastError;
    lastError = error;
    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    
    int leftSpeed = (baseSpeed - PIDvalue) * speedFactor;
    int rightSpeed = (baseSpeed + PIDvalue) * speedFactor;
    
    // Constrain speeds to valid range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    
    Serial.print("Left Speed: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right Speed: ");
    Serial.println(rightSpeed);
    
    // Sharp Turns Handling: If an outer sensor (active when on the black line, i.e. reading 0)
    if (digitalRead(sensorPins[0]) == 0 || digitalRead(sensorPins[NUM_SENSORS - 1]) == 0) {
        leftSpeed *= turnSpeedMultiplier;
        rightSpeed *= turnSpeedMultiplier;
    }
    
    // Curves Handling: If the second sensor from either end is active (0)
    if (digitalRead(sensorPins[1]) == 0 || digitalRead(sensorPins[NUM_SENSORS - 2]) == 0) {
        leftSpeed = (leftSpeed * turnSpeedMultiplier) / 1.3;
        rightSpeed = (rightSpeed * turnSpeedMultiplier) / 1.3;
    }
    
    // Obstacle (Small Bar) Handling: If the center and its adjacent sensors detect the line (0)
    if (sensorValues[3] == 0 && sensorValues[2] == 0 && sensorValues[4] == 0) {
        leftSpeed = baseSpeed * speedFactor;
        rightSpeed = baseSpeed * speedFactor;
    }
    
    // Apply speeds to motors
    motorLeft.setSpeed(leftSpeed);
    motorRight.setSpeed(rightSpeed);
    motorLeft.run(FORWARD);
    motorRight.run(FORWARD);
}

// Read sensor values and compute weighted position
// Now, a sensor is considered "active" (detecting the black line) when it reads 0.
int readSensors() {
    int weightedSum = 0;
    int activeCount = 0;
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = digitalRead(sensorPins[i]);
        // If sensor reads 0, it is over black (line detected)
        if (sensorValues[i] == 0) {
            weightedSum += i * 1000;  // Multiply by 1000 to maintain scale
            activeCount++;
        }
    }
    // If no sensor is active, assume a centered line
    return (activeCount == 0) ? ((NUM_SENSORS - 1) * 500) : (weightedSum / activeCount);
}
