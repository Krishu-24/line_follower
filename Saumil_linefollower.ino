#include <AFMotor.h>

#define NUM_SENSORS 7

// Sensor configuration:
// Order: Left-most to right-most: L3, L2, L1, Center, R1, R2, R3
// Original mapping: L3 = A4, L2 = A5, L1 = A2, C = A0, R1 = 2 (digital), R2 = A3, R3 = A1
const int sensorPins[NUM_SENSORS] = {A4, A5, A2, A0, 2, A3, A1};
// Boolean array to indicate whether a sensor is analog (true) or digital (false)
const bool sensorIsAnalog[NUM_SENSORS] = {true, true, true, true, false, true, true};
// Sensor readings storage
int sensorValues[NUM_SENSORS];
// Use centered positions (in arbitrary units multiplied by 1000) for weighted averaging.
int sensorPositions[NUM_SENSORS] = {-3, -2, -1, 0, 1, 2, 3};

// Threshold for analog sensors (adjust based on calibration)
const int analogThreshold = 500;

// PID constants (tweak these for smoother behavior)
float Kp = 0.8;
float Ki = 0.001;
float Kd = 1.5;

// PID state variables
int error = 0, lastError = 0;
float P = 0, I = 0, D = 0, PIDvalue = 0;

// Base speed parameters
int baseSpeed = 150;          // Base motor speed before scaling
float speedFactor = 0.27;     // Scale computed speed to motor values (0-255)
float turnSpeedMultiplier = 0.6;  // Reduce speed when sharp turn conditions are detected

// Create motor objects on ports M1 and M2
AF_DCMotor motor1(2);
AF_DCMotor motor2(1);

// Function prototypes
int readSensors();
bool sensorActive(int index);
bool isLostLine();
bool isBar();

void setup() {
  Serial.begin(9600);
  
  // Initialize motors
  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
  
  // For digital sensors, set pin mode. (Analog pins do not need pinMode.)
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (!sensorIsAnalog[i]) {
      pinMode(sensorPins[i], INPUT);
    }
  }
}

void loop() {
  // Read all sensor values and compute a weighted average position
  int position = readSensors();
  
  // --- Special condition handling ---
  if (isBar()) {
    // If a bar (crossing) is detected (most sensors active), ignore the error so the robot goes straight.
    error = 0;
  } else if (isLostLine()) {
    // If no sensor detects the line, assume a lost line and default error to 0.
    error = 0;
  } else {
    // Normal line-following: we want to keep the center (position 0) aligned with the line.
    error = 0 - position;
  }
  
  // --- PID calculations ---
  P = error;
  I += error;
  D = error - lastError;
  lastError = error;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  
  // Compute preliminary motor speeds
  int leftSpeed = (baseSpeed - PIDvalue) * speedFactor;
  int rightSpeed = (baseSpeed + PIDvalue) * speedFactor;
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // --- Adjust speeds for sharp turns ---
  // If either outer sensor (leftmost or rightmost) is active, reduce speed.
  if (sensorActive(0) || sensorActive(NUM_SENSORS - 1)) {
    leftSpeed = leftSpeed * turnSpeedMultiplier;
    rightSpeed = rightSpeed * turnSpeedMultiplier;
  }
  // Also check the next-to-outer sensors and further reduce speed slightly.
  if (sensorActive(1) || sensorActive(NUM_SENSORS - 2)) {
    leftSpeed = (leftSpeed * turnSpeedMultiplier) / 1.3;
    rightSpeed = (rightSpeed * turnSpeedMultiplier) / 1.3;
  }
  
  // --- Drive motors ---
  motor1.setSpeed(leftSpeed);
  motor2.setSpeed(rightSpeed);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  
  // Debug output for monitoring sensor readings and PID error.
  Serial.print("Sensors: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.print(" | Error: ");
  Serial.println(error);
  
  delay(10); // Short delay to stabilize loop timing
}

// --------------------------------------------------------------------------
// readSensors: Reads all sensors, computes a weighted average position.
// If no sensor is active, returns 0 (center). Otherwise, the average is based on the sensorPositions.
int readSensors() {
  long weightedSum = 0;
  int activeCount = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int reading = 0;
    if (sensorIsAnalog[i]) {
      reading = analogRead(sensorPins[i]);
    } else {
      // For digital sensors (active low): LOW (0) means active, HIGH means inactive.
      reading = (digitalRead(sensorPins[i]) == LOW) ? 0 : 1023;
    }
    sensorValues[i] = reading;
    
    // Check if the sensor is considered "active" (detecting the dark line)
    if (sensorActive(i)) {
      weightedSum += sensorPositions[i] * 1000; // Multiply factor for resolution
      activeCount++;
    }
  }
  
  if (activeCount == 0) {
    // No sensor is active (lost line) – return 0 to help keep the robot moving straight.
    return 0;
  }
  return weightedSum / activeCount;
}

// --------------------------------------------------------------------------
// sensorActive: Returns true if the sensor at the given index detects the line.
// For analog sensors, the reading must be below the threshold.
// For digital sensors, active means the reading is 0.
bool sensorActive(int index) {
  if (sensorIsAnalog[index]) {
    return sensorValues[index] < analogThreshold;
  } else {
    return sensorValues[index] == 0;
  }
}

// --------------------------------------------------------------------------
// isLostLine: Returns true if none of the sensors detect the line.
bool isLostLine() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorActive(i)) {
      return false;
    }
  }
  return true;
}

// --------------------------------------------------------------------------
// isBar: Returns true if the pattern suggests a crossing bar.
// Here, if 60% or more of the sensors are active at once, we treat it as a bar
// and choose to ignore the deviation, letting the robot go straight.
bool isBar() {
  int activeCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (sensorActive(i)) {
      activeCount++;
    }
  }
  if (activeCount >= (NUM_SENSORS * 0.6)) {
    return true;
  }
  return false;
}
