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

// PID Constants (Tuned for better control)
float Kp = 0.9;     // Proportional (Increase if slow to respond)
float Ki = 0.001;   // Integral (Handles steady-state errors)
float Kd = 3.5;     // Derivative (Reduces oscillations)

// PID Variables
int error, lastError = 0;
int P, I, D, PIDvalue;
int baseSpeed = 255;     // Increased for better power (Full Speed)
float speedFactor = 0.55; // Adjusted for smoother control

// Speed control during sharp turns
float turnSpeedMultiplier = 0.5;  // Slow down for sharp turns

void setup() {
    Serial.begin(115200);
    
    // Initialize motors and ensure they are running
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
    
    // Ensure speed values remain in valid range
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    // Debugging sensor readings (Optional)
    Serial.print("Left Speed: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right Speed: ");
    Serial.println(rightSpeed);

    // **Sharp Turns Handling**  
    // Check if outer sensors (active LOW) are detecting the line
    if (digitalRead(sensorPins[0]) == 0 || digitalRead(sensorPins[NUM_SENSORS - 1]) == 0) {
        leftSpeed *= turnSpeedMultiplier;
        rightSpeed *= turnSpeedMultiplier;
    }

    // **Curves Handling**  
    // Check if next-to-outer sensors are active (detecting the line)
    if (digitalRead(sensorPins[1]) == 0 || digitalRead(sensorPins[NUM_SENSORS - 2]) == 0) {
        leftSpeed = (leftSpeed * turnSpeedMultiplier) / 1.3;
        rightSpeed = (rightSpeed * turnSpeedMultiplier) / 1.3;
    }

    // **Obstacle (small bar) Handling**  
    // If the center and adjacent sensors are active, assume a bar and move straight
    if (sensorValues[3] == 0 && sensorValues[2] == 0 && sensorValues[4] == 0) {
        leftSpeed = baseSpeed * speedFactor;
        rightSpeed = baseSpeed * speedFactor;
    }

    // Apply speed values to motors
    motorLeft.setSpeed(leftSpeed);
    motorRight.setSpeed(rightSpeed);
    motorLeft.run(FORWARD);
    motorRight.run(FORWARD);
}

// Read sensor values and determine position
int readSensors() {
    int weightedSum = 0;
    int sum = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        sensorValues[i] = digitalRead(sensorPins[i]);
        if (sensorValues[i] == 0) {  // Active LOW sensors: 0 means active
            weightedSum += i * 1000;
            sum++;
        }
    }
    return (sum == 0) ? ((NUM_SENSORS - 1) * 500) : (weightedSum / sum);
}
