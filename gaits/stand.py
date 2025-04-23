import time
import math
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

i2c = busio.I2C(SCL, SDA)

# Initialize PCA9685 module
pca = PCA9685(i2c)
pca.frequency = 50  # Standard servo frequency is 50 Hz

# Function to convert angle to duty cycle pulse length
def set_servo_angle(channel, angle):
    # Servo pulse range typically goes from 500us to 2500us
    # For PCA9685, 1 tick = (1 / frequency) / 4096 seconds
    # For 50 Hz, period = 20 ms, so each tick = 20 ms / 4096 ? 4.88 us
    min_pulse = 500  # Minimum pulse length in microseconds
    max_pulse = 2500  # Maximum pulse length in microseconds

    pulse_length = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
    duty_cycle = int((pulse_length / 20000) * 65535)  # Convert to 16-bit duty cycle for PCA9685
    
    pca.channels[channel].duty_cycle = duty_cycle

# Set the first servo (channel 0) to 0 degrees

# Give some time for the servo to move
time.sleep(1)

def solve_angle_c(a, b, c):
    # Apply the Law of Cosines formula to solve for cos(C)
    numerator = a**2 + b**2 - c**2
    denominator = 2 * a * b
    
    # Prevent any possible domain error due to floating-point arithmetic
    cos_C = min(1, max(-1, numerator / denominator))
    
    # Calculate the angle in radians and convert to degrees
    angle_C_radians = math.acos(cos_C)
    angle_C_degrees = math.degrees(angle_C_radians)
    
    return angle_C_degrees
    
def calculate_distance(x1,y1):
    distance = math.sqrt((x1)**2 +(y1)**2)
    return distance
    
while True:
	angleval = 30
	elbowval = 5
	set_servo_angle(5, 0)
	set_servo_angle(4, 180)
	set_servo_angle(10, 180)
	set_servo_angle(11, 0)

	set_servo_angle(6, (90 -((180-angleval)/2)))
	set_servo_angle(9, 180 - elbowval)


	set_servo_angle(0,(90 +((180-angleval)/2)))
	set_servo_angle(1, elbowval)

	set_servo_angle(7, (90 -((180-angleval)/2)) )
	set_servo_angle(8, 180- elbowval)

	set_servo_angle(2,(90 +((180-angleval)/2)) )
	set_servo_angle(3, elbowval)
            




# Optionally, de-initialize the PCA9685
pca.deinit()
