#include <AFMotor.h>

// Create motor objects on ports M1 and M2
AF_DCMotor motor1(2);
AF_DCMotor motor2(1);

// Sensor configuration
#define NUM_SENSORS 7
const int C = A0;  
const int R3 = A1;  
const int L1 = A2;  
const int R2 = A3;   
const int L3 = A4;  
const int L2 = A5;  
const int R1 = A6;  // Changed from 2 to A6 to avoid conflicts

const int sensorPins[NUM_SENSORS] = {L3, L2, L1, C, R1, R2, R3};
int sensorValues[NUM_SENSORS];

// PID constants
float Kp = 0.7;
float Ki = 0.0003;
float Kd = 6.0;

// PID variables
int error, lastError = 0;
int P, I, D, PIDvalue;
int baseSpeed = 150;
float speedFactor = 0.27;

// Speed control multipliers
float turnSpeedMultiplier = 0.4; // More aggressive speed reduction at turns
bool searchingForLine = false;

void setup() {
    Serial.begin(115200);
    
    motor1.setSpeed(0);
    motor1.run(RELEASE);
    motor2.setSpeed(0);
    motor2.run(RELEASE);
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        pinMode(sensorPins[i], INPUT);
    }
}

void loop() {
    int position = readSensors();
    error = (NUM_SENSORS - 1) * 500 - position;
    
    // If center sensor (C) doesn't detect a line, enter search mode
    if (digitalRead(C) == 1) {
        searchForLine();
        return;
    }
    
    searchingForLine = false; // Reset search mode when line is found
    
    // PID calculations
    P = error;
    I += error;
    I = constrain(I, -1000, 1000); // Prevent integral windup
    D = error - lastError;
    lastError = error;
    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    
    int leftSpeed = (baseSpeed - PIDvalue) * speedFactor;
    int rightSpeed = (baseSpeed + PIDvalue) * speedFactor;
    
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);
    
    // Reduce speed if outer sensors detect a turn
    if (digitalRead(sensorPins[0]) == 0 || digitalRead(sensorPins[NUM_SENSORS - 1]) == 0) {
        leftSpeed *= turnSpeedMultiplier;
        rightSpeed *= turnSpeedMultiplier;
    }
    
    if (digitalRead(sensorPins[1]) == 0 || digitalRead(sensorPins[NUM_SENSORS - 2]) == 0) {
        leftSpeed *= (turnSpeedMultiplier / 1.2);
        rightSpeed *= (turnSpeedMultiplier / 1.2);
    }
    
    // Apply speed values
    motor1.setSpeed(leftSpeed);
    motor2.setSpeed(rightSpeed);
    motor1.run(FORWARD);
    motor2.run(FORWARD);
}

int readSensors() {
    int weightedSum = 0;
    int sum = 0;
    int weights[NUM_SENSORS] = {-3, -2, -1, 0, 1, 2, 3};
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = digitalRead(sensorPins[i]);
        if (sensorValues[i] == 0) {
            weightedSum += weights[i] * 1000;
            sum++;
        }
    }
    
    if (sum == 0) {
        motor1.setSpeed(0);
        motor2.setSpeed(0);
        delay(100);
        return (NUM_SENSORS - 1) * 500; // Assume farthest right position to correct
    }
    
    return weightedSum / sum;
}

void searchForLine() {
    if (!searchingForLine) {
        searchingForLine = true;
        Serial.println("Lost line! Searching...");
    }
    
    motor1.setSpeed(100);
    motor2.setSpeed(100);
    
    motor1.run(BACKWARD);
    motor2.run(FORWARD);
    delay(200);
}
