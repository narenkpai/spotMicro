#include <ESP32Servo.h>

// Define the servo objects
Servo myservo;
Servo myservo1;

// Define the servo pins
int servoPin = 18;
int servoPin1 = 16;

// Minimum and maximum position values
int minPosition = 100;
int maxPosition = 300;

// Step size for incrementing or decrementing the position
int stepSize = 1;

// Current position value
int position = minPosition;

// Variable to control the direction of movement
bool increasing = true;

void setup() {
  // Attach the servos to the defined pins
  myservo.attach(servoPin);
  myservo1.attach(servoPin1);
  
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("MG996R Servo Control Looping from 100 to 300.");
}

void loop() {
  // Calculate angle C based on the current position
  float angleC = calculateAngleC(130, 130, position);
  
  // Calculate theta1
  int theta1 = (180 - angleC) / 2;

  // Move the servos based on the calculated angles
  if (angleC > 90) {
    myservo.write(angleC);
  } else {
    myservo.write(90);
  }
  if(theta1 < 50){
    myservo1.write(80 + theta1);
  }else{
    myservo1.write(130);
  }
   // Print the current position and angles for debugging
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(", Angle C: ");
  Serial.print(angleC);
  Serial.print(", Theta1: ");
  Serial.println(theta1);
  
  // Update the position
  if (increasing) {
    position += stepSize;
    if (position >= maxPosition) {
      position = maxPosition;
      increasing = false;  // Start decreasing
    }
  } else {
    position -= stepSize;
    if (position <= minPosition) {
      position = minPosition;
      increasing = true;  // Start increasing
    }
  }

  // Add a small delay for smoother movement
}

// Function to calculate angle C using the Law of Cosines
float calculateAngleC(float a, float b, float c) {
  // Calculate the cosine of angle C
  float cosC = (a * a + b * b - c * c) / (2.0 * a * b);
  
  // Check for any floating-point inaccuracies
  if (cosC > 1.0) cosC = 1.0;
  if (cosC < -1.0) cosC = -1.0;

  // Calculate angle C in radians
  float angleC_rad = acos(cosC);

  // Convert the angle to degrees
  float angleC_deg = degrees(angleC_rad);

  return angleC_deg;
}
