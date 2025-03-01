#include <ESP32Servo.h>

// Servo control parameters
const int SHOULDER_PIN = 18;  // GPIO pin for shoulder servo
const int ELBOW_PIN = 16;     // GPIO pin for elbow servo

// Servo objects
Servo shoulderServo;
Servo elbowServo;

// Function to constrain the servo position to the allowed range (100 to 180 degrees)
int constrainServoPosition(int angle) {
  return constrain(angle, 100, 180);
}

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  
  // Attach the servos to the defined pins
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);

  // Set initial position for both servos within the allowed range
  int initialPosition = 140; // Starting position within 100-180 range
  shoulderServo.write(constrainServoPosition(initialPosition));
  elbowServo.write(constrainServoPosition(initialPosition));

  Serial.println("Both servos attached and set to initial position.");
  delay(1000);  // Wait for the servos to reach initial position
}

void loop() {
  // Sweep both servos from 100 to 180 degrees and back
  for (int pos = 100; pos <= 180; pos += 5) {  // Increment by 5 for smoother movement
    shoulderServo.write(constrainServoPosition(pos));
    elbowServo.write(constrainServoPosition(pos));
    Serial.print("Both servos set to position: ");
    Serial.println(pos);
    delay(100);  // Adjust delay as needed for smoother motion
  }

  delay(1000);  // Pause at the end of the range

  for (int pos = 180; pos >= 100; pos -= 5) {  // Decrement by 5 for smoother movement
    shoulderServo.write(constrainServoPosition(pos));
    elbowServo.write(constrainServoPosition(pos));
    Serial.print("Both servos set to position: ");
    Serial.println(pos);
    delay(100);  // Adjust delay as needed for smoother motion
  }

  delay(1000);  // Pause before repeating the loop
}
