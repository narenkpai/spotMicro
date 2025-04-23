import time
import math
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import cv2
import numpy as np
import ArducamDepthCamera as ac
import matplotlib.pyplot as plt

i2c = busio.I2C(SCL, SDA)

# Initialize PCA9685 module
pca = PCA9685(i2c)
pca.frequency = 50  # Standard servo frequency is 50 Hz
filtered_depthVal = None  
alpha = 0.2  # Smoothing factor for filtering (adjust between 0.1 - 0.3)

class UserRect:
    """Class to store and manage user-selected rectangles"""
    def __init__(self):
        self.start_x, self.start_y = 0, 0
        self.end_x, self.end_y = 0, 0

    @property
    def rect(self):
        return self.start_x, self.start_y, self.end_x - self.start_x, self.end_y - self.start_y

    @property
    def slice(self):
        return slice(self.start_y, self.end_y), slice(self.start_x, self.end_x)

    @property
    def empty(self):
        return self.start_x == self.end_x and self.start_y == self.end_y

# Global objects
confidence_value = 30
selectRect, followRect = UserRect(), UserRect()

def getPreviewRGB(preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    """Apply confidence filtering to the preview image"""
    preview = np.nan_to_num(preview)
    preview[confidence < confidence_value] = (0, 0, 0)
    return preview

def on_mouse(event, x, y, flags, param):
    """Mouse event callback for rectangle selection"""
    global selectRect, followRect
    if event == cv2.EVENT_LBUTTONUP:
        selectRect.start_x, selectRect.start_y = x - 4, y - 4
        selectRect.end_x, selectRect.end_y = x + 4, y + 4
    else:
        followRect.start_x, followRect.start_y = x - 4, y - 4
        followRect.end_x, followRect.end_y = x + 4, y + 4

def on_confidence_changed(value):
    """Confidence slider callback"""
    global confidence_value
    confidence_value = value

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
    
def resolveFrame():
    frame = cam.requestFrame(2000)
    if frame is not None and isinstance(frame, ac.DepthData):
        # Retrieve depth and confidence data
        depth_buf = frame.getDepthData()
        confidence_buf = frame.getConfidenceData()
        depth_buf = np.clip(depth_buf, 0, 2000)  # Clip depth range

        # Convert depth data to color map
        result_image = (depth_buf * (255.0 / 50)).astype(np.uint8)
        result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_RAINBOW)
        result_image = getPreviewRGB(result_image, confidence_buf)

        # Normalize and show confidence buffer
        cv2.normalize(confidence_buf, confidence_buf, 1, 0, cv2.NORM_MINMAX)
        cv2.imshow("preview_confidence", cv2.rotate(confidence_buf, cv2.ROTATE_180))

        # Define center region for depth calculation
        frame_height, frame_width = depth_buf.shape
        center_x, center_y = frame_width // 2, frame_height // 2
        rect_size = 10  # Size of the region to analyze
        center_slice = (
            slice(center_y - rect_size, center_y + rect_size),
            slice(center_x - rect_size, center_x + rect_size),
        )
        # Draw center rectangle
        cv2.rectangle(result_image, 
                        (center_x - rect_size, center_y - rect_size), 
                        (center_x + rect_size, center_y + rect_size), 
                        (0, 0, 0), 2)

        # Compute and filter depth value
        depthVal = np.mean(depth_buf[center_slice])
        depthVal = min(depthVal, 250)  # Cap max depth at 250

        if filtered_depthVal is None:
            filtered_depthVal = depthVal  # Initialize on first run
        else:
            filtered_depthVal = alpha * depthVal + (1 - alpha) * filtered_depthVal  # Apply smoothing

        print("Filtered Center Rect distance:", filtered_depthVal)
        return filtered_depthVal

base_length = 30
height = 10
pointsPerSide = 15
totalPoints = 45
vertical_distance = 10
k = 1
while True:
    if k == 1:  # backleft
        # Adjust the vertical distance using the resolved frame value
        adjusted_vertical_distance = vertical_distance - (resolveFrame() / 10)
        
        # Define triangle vertices with point 2 at the origin
        p2 = np.array([0, 0])
        p1 = np.array([-base_length/2, -adjusted_vertical_distance])
        p3 = np.array([ base_length/2, -adjusted_vertical_distance])
        
        # Generate coordinate points along each side of the triangle
        side1_points = np.column_stack((
            np.linspace(p1[0], p2[0], pointsPerSide),
            np.linspace(p1[1], p2[1], pointsPerSide)
        ))
        side2_points = np.column_stack((
            np.linspace(p2[0], p3[0], pointsPerSide),
            np.linspace(p2[1], p3[1], pointsPerSide)
        ))
        side3_points = np.column_stack((
            np.linspace(p3[0], p1[0], pointsPerSide),
            np.linspace(p3[1], p1[1], pointsPerSide)
        ))
        
        # Concatenate all side points into a single array
        all_points = np.concatenate((side2_points, side1_points, side3_points), axis=0)
        
        # Iterate over each point and process servo commands
        for pt in all_points:
            x, y = pt[0], pt[1]
            print("x val ", x)
            print("y val ", y)
            distanceval = calculate_distance(x - 15, y)
            angleval = solve_angle_c(130, 130, distanceval)
            # Caution: ensure y is not zero to avoid division error
            subsubAngle = math.atan(x / y) if y != 0 else 0
            subAngle = math.degrees(subsubAngle)
            print("tan angle ", subAngle)
            
            set_servo_angle(5, 0)
            set_servo_angle(4, 180)
            set_servo_angle(10, 180)
            set_servo_angle(11, 0)
            set_servo_angle(6, (90 - ((180 - angleval) / 2)) + subAngle / 2)
            set_servo_angle(9, 180 - angleval)
    k = 2
    if k == 2:  # backright
        # Adjust the vertical distance using the resolved frame value
        adjusted_vertical_distance = vertical_distance - (resolveFrame() / 10)
        
        # Define triangle vertices with point 2 at the origin
        p2 = np.array([0, 0])
        p1 = np.array([-base_length/2, -adjusted_vertical_distance])
        p3 = np.array([ base_length/2, -adjusted_vertical_distance])
        
        # Generate coordinate points along each side of the triangle
        side1_points = np.column_stack((
            np.linspace(p1[0], p2[0], pointsPerSide),
            np.linspace(p1[1], p2[1], pointsPerSide)
        ))
        side2_points = np.column_stack((
            np.linspace(p2[0], p3[0], pointsPerSide),
            np.linspace(p2[1], p3[1], pointsPerSide)
        ))
        side3_points = np.column_stack((
            np.linspace(p3[0], p1[0], pointsPerSide),
            np.linspace(p3[1], p1[1], pointsPerSide)
        ))
        
        # Concatenate all side points into a single array
        all_points = np.concatenate((side2_points, side1_points, side3_points), axis=0)
        
        # Iterate over each point and process servo commands
        for pt in all_points:
            x, y = pt[0], pt[1]
            print("x val ", x)
            print("y val ", y)
            
            distanceval = calculate_distance(x - 15, y - 10)
            angleval = solve_angle_c(130, 130, distanceval)
            subsubAngle = math.atan(x / y) if y != 0 else 0
            subAngle = math.degrees(subsubAngle)
            print("tan angle ", subAngle)
            
            set_servo_angle(5, 0)
            set_servo_angle(4, 180)
            set_servo_angle(10, 180)
            set_servo_angle(11, 0)
            set_servo_angle(0, (90 + ((180 - angleval) / 2)) - subAngle / 2)
            set_servo_angle(1, angleval)
    k = 3
    if k == 3:  # frontright
        # Adjust the vertical distance using the resolved frame value
        adjusted_vertical_distance = vertical_distance - (resolveFrame() / 10)
        
        # Define triangle vertices with point 2 at the origin
        p2 = np.array([0, 0])
        p1 = np.array([-base_length/2, -adjusted_vertical_distance])
        p3 = np.array([ base_length/2, -adjusted_vertical_distance])
        
        # Generate coordinate points along each side of the triangle
        side1_points = np.column_stack((
            np.linspace(p1[0], p2[0], pointsPerSide),
            np.linspace(p1[1], p2[1], pointsPerSide)
        ))
        side2_points = np.column_stack((
            np.linspace(p2[0], p3[0], pointsPerSide),
            np.linspace(p2[1], p3[1], pointsPerSide)
        ))
        side3_points = np.column_stack((
            np.linspace(p3[0], p1[0], pointsPerSide),
            np.linspace(p3[1], p1[1], pointsPerSide)
        ))
        
        # Concatenate all side points into a single array
        all_points = np.concatenate((side2_points, side1_points, side3_points), axis=0)
        
        # Iterate over each point and process servo commands
        for pt in all_points:
            x, y = pt[0], pt[1]
            print("x val ", x)
            print("y val ", y)
            
            distanceval = calculate_distance(x - 15, y - 10)
            angleval = solve_angle_c(130, 130, distanceval)
            subsubAngle = math.atan(x / y) if y != 0 else 0
            subAngle = math.degrees(subsubAngle)
            print("tan angle ", subAngle)
            
            set_servo_angle(5, 0)
            set_servo_angle(4, 180)
            set_servo_angle(10, 180)
            set_servo_angle(11, 0)
            set_servo_angle(7, (90 - ((180 - angleval) / 2)) + subAngle / 2)
            set_servo_angle(8, 180 - angleval)
    k = 4
    if k == 4:  # frontleft
        # Adjust the vertical distance using the resolved frame value
        adjusted_vertical_distance = vertical_distance - (resolveFrame() / 10)
        
        # Define triangle vertices with point 2 at the origin
        p2 = np.array([0, 0])
        p1 = np.array([-base_length/2, -adjusted_vertical_distance])
        p3 = np.array([ base_length/2, -adjusted_vertical_distance])
        
        # Generate coordinate points along each side of the triangle
        side1_points = np.column_stack((
            np.linspace(p1[0], p2[0], pointsPerSide),
            np.linspace(p1[1], p2[1], pointsPerSide)
        ))
        side2_points = np.column_stack((
            np.linspace(p2[0], p3[0], pointsPerSide),
            np.linspace(p2[1], p3[1], pointsPerSide)
        ))
        side3_points = np.column_stack((
            np.linspace(p3[0], p1[0], pointsPerSide),
            np.linspace(p3[1], p1[1], pointsPerSide)
        ))
        
        # Concatenate all side points into a single array
        all_points = np.concatenate((side2_points, side1_points, side3_points), axis=0)
        
        # Iterate over each point and process servo commands
        for pt in all_points:
            x, y = pt[0], pt[1]
            print("x val ", x)
            print("y val ", y)
            
            distanceval = calculate_distance(x - 15, y + 40)
            angleval = solve_angle_c(130, 130, distanceval)
            subsubAngle = math.atan(x / y) if y != 0 else 0
            subAngle = math.degrees(subsubAngle)
            print("tan angle ", subAngle)
            
            set_servo_angle(5, 0)
            set_servo_angle(4, 180)
            set_servo_angle(10, 180)
            set_servo_angle(11, 0)
            set_servo_angle(2, (90 + ((180 - angleval) / 2)) - subAngle / 2)
            set_servo_angle(3, angleval)
            k = 1

# Optionally, de-initialize the PCA9685
pca.deinit()