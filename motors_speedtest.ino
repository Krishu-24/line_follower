#include <AFMotor.h>

// Initialize motors on M1 and M2
AF_DCMotor leftMotor(1);  
AF_DCMotor rightMotor(2);  

void setup() {
    // Wait for 2 seconds before starting
    delay(2000);

    // Gradually increase speed from 0 to 255
    for (int speed = 0; speed <= 255; speed += 5) {  
        leftMotor.setSpeed(speed);
        rightMotor.setSpeed(speed);
        leftMotor.run(BACKWARD);  // Adjust if needed
        rightMotor.run(BACKWARD); // Adjust if needed
        delay(50);  // Small delay for smooth acceleration
    }

    // Run at full speed for 5 seconds
    delay(5000);

    // Stop motors
    leftMotor.run(RELEASE);
    rightMotor.run(RELEASE);
}

void loop() {
    // Do nothing after stopping
}
