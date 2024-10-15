import time
import board
import busio
from digitalio import DigitalInOut
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

i2c = busio.I2C(3, 2)

pca = PCA9685(i2c)
pca.frequency = 50

# Create servo objects on channel 0 and channel 1 of the PCA9685
servo_0 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
servo_1 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)

# Function to move a servo to a specific angle
def set_servo_angle(servo_channel, angle):
    if 0 <= angle <= 110:
        servo_channel.angle = angle
        print(f"Servo moved to {angle} degrees")
    else:
        print("Angle out of range. Must be between 0 and 180.")

# Example usage
try:
    while True:
        # Move both servos from 0 to 180 degrees
        for angle in range(0, 110, 10):
            set_servo_angle(servo_0, angle)  # Control servo on channel 0
            set_servo_angle(servo_1, angle)  # Control servo on channel 1
            time.sleep(0.5)
        
        # Move both servos back from 180 to 0 degrees
        for angle in range(110, -1, -10):
            set_servo_angle(servo_0, angle)
            set_servo_angle(servo_1, angle)
            time.sleep(0.5)
except KeyboardInterrupt:
    print("Program stopped")

# Ensure to release the I2C resources when done
pca.deinit()
