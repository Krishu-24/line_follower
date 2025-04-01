#include <AFMotor.h>

// Define motors
AF_DCMotor motorLeft(1);
AF_DCMotor motorRight(2);

// Define sensor pins
const int C = A0;  
const int R3 = A1;  
const int L1 = A2;  
const int R2 = A3;   
const int L3 = A4;  
const int L2 = A5;  
const int R1 = 2;  

const int NUM_SENSORS = 7;
const int sensorPins[NUM_SENSORS] = {L3, L2, L1, C, R1, R2, R3};  

// Sensor position weights (-3 to +3, left to right)
const int positionWeights[NUM_SENSORS] = {-3, -2, -1, 0, 1, 2, 3};  

int sensorValues[NUM_SENSORS];

// PID Constants
float Kp = 18.0;  // Proportional gain
float Ki = 0.0;   // Integral gain (optional)
float Kd = 10.0;  // Derivative gain

// Speed settings
int baseSpeed = 170; // Base speed on straight paths
int minSpeed = 100;
int maxSpeed = 230;  // Maximum allowed speed

// PID Variables
float error = 0, previousError = 0, integral = 0;

// Failsafe variables for dashed line gaps
int noLineCounter = 0;
const int noLineThreshold = 15; // Number of consecutive loops allowed with no detection

// Function to read sensors and calculate error
int getError() {
    int weightedSum = 0, sum = 0;
    bool lineDetected = false;

    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = analogRead(sensorPins[i]);
        int value = sensorValues[i] > 500 ? 1 : 0;  // Adjust threshold if needed

        weightedSum += value * positionWeights[i];
        sum += value;

        if (value == 1)
            lineDetected = true;
    }

    if (!lineDetected) {
        noLineCounter++; // Increase counter if no sensor sees the line
        // If the gap persists too long, return a special error value (0) to trigger a slowdown/stop
        if (noLineCounter > noLineThreshold) {
            return 0;
        }
        // Otherwise, use the last known error for short gaps
        return previousError;
    } else {
        noLineCounter = 0; // Reset counter when the line is detected
    }

    // Compute error using weighted average
    return weightedSum / sum;
}

void setup() {
    Serial.begin(9600);
    motorLeft.setSpeed(baseSpeed);
    motorRight.setSpeed(baseSpeed);
    motorLeft.run(FORWARD);
    motorRight.run(FORWARD);
}

void loop() {
    error = getError();

    // PID calculations
    float P = error * Kp;
    integral += error;
    float I = integral * Ki;
    float D = (error - previousError) * Kd;
    int correction = P + I + D;

    // Adaptive speed: slow down if a sharp turn is needed
    int leftSpeed, rightSpeed;
    if (abs(error) > 2) { 
        // Slow down during sharp turns
        leftSpeed = constrain(baseSpeed - correction, minSpeed, maxSpeed - 50);
        rightSpeed = constrain(baseSpeed + correction, minSpeed, maxSpeed - 50);
    } else { 
        // On straighter paths, maintain base speed
        leftSpeed = constrain(baseSpeed - correction, minSpeed, maxSpeed);
        rightSpeed = constrain(baseSpeed + correction, minSpeed, maxSpeed);
    }

    // If the gap has been too long (no line detected), slow down further or stop
    if (noLineCounter > noLineThreshold) {
        leftSpeed = rightSpeed = minSpeed; // Slow or stop to prevent runaway behavior
    }

    motorLeft.setSpeed(leftSpeed);
    motorRight.setSpeed(rightSpeed);
    motorLeft.run(FORWARD);
    motorRight.run(FORWARD);

    previousError = error;
}
