#include <AFMotor.h>

// Motor connections using Adafruit Motor Shield v1
AF_DCMotor motor1(2);  // Right motor
AF_DCMotor motor2(1);  // Left motor

// Sensor configuration
#define NUM_SENSORS 7
const int C = A0;
const int R3 = A1;
const int L1 = A2;
const int R2 = A3;
const int L3 = A4;
const int L2 = A5;
const int R1 = 2;

const int sensorPins[NUM_SENSORS] = { L3, L2, L1, C, R1, R2, R3 };
int sensorValues[NUM_SENSORS];

int revspeedvar=1.2; //divider for reverse speed multiplier  set to 1 if does not work
// PID constants
float Kp = 1.0;    // Proportional
float Ki = 0.0001;  // Integral
float Kd = 4.5;     // Derivative

// PID variables
int error, lastError = 0;
int P, I, D, PIDvalue;
int baseSpeed = 150;
float speedFactor = 0.29;//0.3  // General speed scaling

// Turn factors (low for inner, high for outer)
float turnFactorInner = 0.3;
float turnFactorMiddle = 0.4;//0.4;
float turnFactorOuter = 0.5;//0.5;

void setup() {
  Serial.begin(115200);

  Serial.println("\n-------------------------------------------------------------------------------------------------");
  Serial.println("| LeftSpeed | RightSpeed | L_Inner | R_Inner | L_Middle | R_Middle | L_Outer | R_Outer |");
  Serial.println("-------------------------------------------------------------------------------------------------");

  // Initialize motors
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor1.run(RELEASE);
  motor2.run(RELEASE);

  // Initialize sensors
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

  // Calculate speed
  int leftSpeed = baseSpeed - PIDvalue;
  int rightSpeed = baseSpeed + PIDvalue;

  // Determine direction
  bool leftBackward = leftSpeed < 0;
  bool rightBackward = rightSpeed < 0;

  // Take absolute speed values
  leftSpeed = constrain(abs(leftSpeed) * speedFactor, 0, 255);
  rightSpeed = constrain(abs(rightSpeed) * speedFactor, 0, 255);
  I = constrain(I, -1000, 1000); // Prevent excessive accumulation

  if (error == 0) {
    leftSpeed = rightSpeed = baseSpeed * speedFactor;  // Ensure perfect straight-line movement
  }

  Serial.print("| ");
  Serial.print(leftSpeed);
  Serial.print("\t | ");
  Serial.print(rightSpeed);
  Serial.print("\t  | ");

  Serial.print(leftSpeed * turnFactorInner);  // Left Inner
  Serial.print("\t  | ");
  Serial.print(rightSpeed * turnFactorInner);  // Right Inner

  Serial.print("\t  | ");
  Serial.print(leftSpeed * turnFactorMiddle);  // Left Middle
  Serial.print("\t  | ");
  Serial.print(rightSpeed * turnFactorMiddle);  // Right Middle

  Serial.print("\t  | ");
  Serial.print(leftSpeed * turnFactorOuter);  // Left Outer
  Serial.print("\t  | ");
  Serial.print(rightSpeed * turnFactorOuter);  // Right Outer

  Serial.println("\t |");

  // delay(500);

  // Apply movement
  applyMotorControl(leftSpeed, rightSpeed, leftBackward, rightBackward);
}

void applyMotorControl(int leftSpeed, int rightSpeed, bool leftBackward, bool rightBackward) {
  // Handle extreme turns using outermost sensors (L3 and R3) - strongest correction
  if (sensorValues[0] == 1) {  // Leftmost sensor triggered (L3) -> extreme correction (turn right)
    setMotorSpeeds((rightSpeed * turnFactorOuter)/revspeedvar, leftSpeed * turnFactorOuter, false, true);
  } else if (sensorValues[6] == 1) {  // Rightmost sensor triggered (R3) -> extreme correction (turn left)
    setMotorSpeeds(rightSpeed * turnFactorOuter, (leftSpeed * turnFactorOuter)/revspeedvar, true, false);
  }
  // Handle middle sensors for 45° turns - moderate correction
  else if (sensorValues[1] == 1) {  // Middle left sensor triggered (L2) -> moderate correction (turn right)
    setMotorSpeeds((rightSpeed * turnFactorMiddle)/revspeedvar, leftSpeed * turnFactorMiddle, false, true);
  } else if (sensorValues[5] == 1) {  // Middle right sensor triggered (R2) -> moderate correction (turn left)
    setMotorSpeeds(rightSpeed * turnFactorMiddle, (leftSpeed * turnFactorMiddle)/revspeedvar, true, false);                   
  }
  // Handle inner sensors for slight correction - weakest correction
  else if (sensorValues[2] == 1) {  // Inner left sensor triggered (L1)
    setMotorSpeeds(rightSpeed * turnFactorInner, leftSpeed * turnFactorInner, false, false);
  } else if (sensorValues[4] == 1) {  // Inner right sensor triggered (R1)
    setMotorSpeeds(rightSpeed * turnFactorInner, leftSpeed * turnFactorInner, false, false);
  } else {  // Normal PID-controlled movement
    setMotorSpeeds(rightSpeed, leftSpeed, rightBackward, leftBackward);
  }
}
//function to set motor speeds and direction.
void setMotorSpeeds(int rightSpeed, int leftSpeed, bool rightBackward, bool leftBackward) {
  // Set motor1 (right motor)
  motor1.setSpeed(rightSpeed);
  motor1.run(rightBackward ? BACKWARD : FORWARD);

  // Set motor2 (left motor)
  motor2.setSpeed(leftSpeed);
  motor2.run(leftBackward ? BACKWARD : FORWARD);
}

int readSensors() {
  int weightedSum = 0;
  int sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
    if (sensorValues[i] == 1) {
      weightedSum += i * 1000;
      sum++;
    }
  }
  return (sum == 0) ? ((NUM_SENSORS - 1) * 500) : (weightedSum / sum);
}
