import time
import math
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Setup I2C and PCA9685
i2c = busio.I2C(3, 2)  # Specify the I2C pins (usually SCL: 3, SDA: 2 for Raspberry Pi)
pca = PCA9685(i2c)
pca.frequency = 50

# Define servo objects for channels 0, 1, 4, 5, 8, 9, 12, and 13
servo_0 = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
servo_1 = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)
servo_4 = servo.Servo(pca.channels[4], min_pulse=500, max_pulse=2500)
servo_5 = servo.Servo(pca.channels[5], min_pulse=500, max_pulse=2500)
servo_8 = servo.Servo(pca.channels[8], min_pulse=500, max_pulse=2500)
servo_9 = servo.Servo(pca.channels[9], min_pulse=500, max_pulse=2500)
servo_12 = servo.Servo(pca.channels[12], min_pulse=500, max_pulse=2500)
servo_13 = servo.Servo(pca.channels[13], min_pulse=500, max_pulse=2500)

# Minimum and maximum position values
min_position = 160
max_position = 270

# Step size for incrementing or decrementing the position
step_size = 1

# Current position value
position = min_position

# Variable to control the direction of movement
increasing = True

def calculate_angle_c(a, b, c):
    """
    Calculate angle C using the Law of Cosines.
    """
    # Calculate the cosine of angle C
    cos_c = (a * a + b * b - c * c) / (2.0 * a * b)
    
    # Clamp cos_c to the range [-1.0, 1.0] to avoid math domain errors
    cos_c = max(min(cos_c, 1.0), -1.0)
    
    # Calculate angle C in radians
    angle_c_rad = math.acos(cos_c)
    
    # Convert the angle to degrees
    angle_c_deg = math.degrees(angle_c_rad)
    
    return angle_c_deg

def set_servo_angle(servo, angle):
    """
    Sets the angle for a given servo object.
    """
    servo.angle = angle

try:
    print("MG996R Servo Control Looping from 100 to 300.")
    
    while True:
        # Calculate angle C based on the current position
        angle_c = calculate_angle_c(130, 130, position)
        
        # Calculate theta1
        theta1 = (180 - angle_c) / 2
        
        # Move all servos based on the calculated angles
        if angle_c > 90:
            for s in [servo_0, servo_4, servo_8, servo_12]:
                set_servo_angle(s, angle_c)
        else:
            for s in [servo_0, servo_4, servo_8, servo_12]:
                set_servo_angle(s, 90)
        
        if theta1 < 50:
            for s in [servo_1, servo_5, servo_9, servo_13]:
                set_servo_angle(s, 80 + theta1)
        else:
            for s in [servo_1, servo_5, servo_9, servo_13]:
                set_servo_angle(s, 130)
        
        # Print the current position and angles for debugging
        print(f"Position: {position}, Angle C: {angle_c:.2f}, Theta1: {theta1:.2f}")
        
        # Update the position
        if increasing:
            position += step_size
            if position >= max_position:
                position = max_position
                increasing = False  # Start decreasing
        else:
            position -= step_size
            if position <= min_position:
                position = min_position
                increasing = True  # Start increasing
        
        # Add a small delay for smoother movement
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Program stopped")

finally:
    # Release the I2C resources
    pca.deinit()
