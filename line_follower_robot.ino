#include <AFMotor.h>

// Create motor objects on ports M1 and M2
AF_DCMotor leftMotor(1);
AF_DCMotor rightMotor(2);

// Sensor pins (mapping remains unchanged)
const int leftOuterPin = A4;   // Left Outer Sensor (L3)
const int leftMiddlePin = A5;  // Left Middle Sensor (L2)
const int leftInnerPin = A2;   // Left Inner Sensor (L1)
const int rightInnerPin = 2;   // Right Inner Sensor (R1, digital)
const int rightMiddlePin = A3; // Right Middle Sensor (R2)
const int rightOuterPin = A1;  // Right Outer Sensor (R3)

// Threshold for analog sensors (adjust based on calibration)
int threshold = 500;

// Speed settings
const int highSpeed = 80;  // Fast speed for straight lines
const int midSpeed = 60;   // Medium speed for slight turns
const int lowSpeed = 40;   // Low speed for sharp turns and entry-loop

void setup() {
  Serial.begin(9600);
  pinMode(leftOuterPin, INPUT);
  pinMode(leftMiddlePin, INPUT);
  pinMode(leftInnerPin, INPUT);
  pinMode(rightInnerPin, INPUT);
  pinMode(rightMiddlePin, INPUT);
  pinMode(rightOuterPin, INPUT);
}

void loop() {
  // Read sensor values
  int LO = (analogRead(leftOuterPin) < threshold);
  int LM = (analogRead(leftMiddlePin) < threshold);
  int LI = (analogRead(leftInnerPin) < threshold);
  int RI = (digitalRead(rightInnerPin) == LOW);
  int RM = (analogRead(rightMiddlePin) < threshold);
  int RO = (analogRead(rightOuterPin) < threshold);

  // Debug prints
  Serial.print("LO: "); Serial.print(LO);
  Serial.print(" | LM: "); Serial.print(LM);
  Serial.print(" | LI: "); Serial.print(LI);
  Serial.print(" | RI: "); Serial.print(RI);
  Serial.print(" | RM: "); Serial.print(RM);
  Serial.print(" | RO: "); Serial.println(RO);

  // ======== ENTRY-LOOP DETECTION (New Code Block) =========
  // If both inner sensors are off, but at least one middle sensor is active,
  // assume we're entering a loop. Slow down and pivot.
  if (!LI && !RI && (LM || RM)) {
    leftMotor.setSpeed(lowSpeed);
    rightMotor.setSpeed(lowSpeed);
    // Decide turning direction based on which middle sensor is active:
    if (LM && !RM) {
      // If only left middle is active, pivot right:
      leftMotor.run(BACKWARD);
      rightMotor.run(FORWARD);
    } else if (RM && !LM) {
      // If only right middle is active, pivot left:
      leftMotor.run(FORWARD);
      rightMotor.run(BACKWARD);
    } else {
      // If both are active (or ambiguous), drive slowly straight:
      leftMotor.run(BACKWARD);
      rightMotor.run(BACKWARD);
    }
    return; // Exit loop early so the normal logic is skipped.
  }
  // ===== End of Entry-Loop Block =====

  // Normal control logic:
  if (LI && RI) {
    // Both inner sensors detect line → Fast speed (straight)
    leftMotor.setSpeed(highSpeed);
    rightMotor.setSpeed(highSpeed);
    leftMotor.run(BACKWARD);
    rightMotor.run(BACKWARD);
  } 
  else if (LI || RI) {
    // Only one inner sensor → Medium speed for turning
    leftMotor.setSpeed(midSpeed);
    rightMotor.setSpeed(midSpeed);
    if (LI) {
      leftMotor.run(BACKWARD);
      rightMotor.run(FORWARD);
    } else {
      leftMotor.run(FORWARD);
      rightMotor.run(BACKWARD);
    }
  }
  else if (LM || RM) {
    // Middle sensors (drifting correction) → Slow speed
    leftMotor.setSpeed(lowSpeed);
    rightMotor.setSpeed(lowSpeed);
    if (LM) {
      leftMotor.run(BACKWARD);
      rightMotor.run(RELEASE);
    } else {
      leftMotor.run(RELEASE);
      rightMotor.run(BACKWARD);
    }
  }
  else if (LO || RO) {
    // Outer sensors (hard correction) → Slow speed
    leftMotor.setSpeed(lowSpeed);
    rightMotor.setSpeed(lowSpeed);
    if (LO) {
      leftMotor.run(BACKWARD);
      rightMotor.run(FORWARD);
    } else {
      leftMotor.run(FORWARD);
      rightMotor.run(BACKWARD);
    }
  }
  else {
    // No sensor detects line → Stop
    leftMotor.run(RELEASE);
    rightMotor.run(RELEASE);
  }
}
