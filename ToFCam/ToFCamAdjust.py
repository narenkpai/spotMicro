import cv2
import numpy as np
import time
import math
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import ArducamDepthCamera as ac

# Initialize I2C for PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50  # Standard servo frequency is 50 Hz

# Global variables for depth filtering
filtered_depthVal = None  
alpha = 0.2  # Smoothing factor for filtering (adjust between 0.1 - 0.3)

def set_servo_angle(channel, angle):
    """Set the servo angle for PCA9685"""
    min_pulse, max_pulse = 500, 2500  # Min and max pulse length in microseconds
    pulse_length = min_pulse + (angle / 180.0) * (max_pulse - min_pulse)
    duty_cycle = int((pulse_length / 20000) * 65535)  # Convert to 16-bit duty cycle
    pca.channels[channel].duty_cycle = duty_cycle

def solve_angle_c(a, b, c):
    """Solve for an angle using the Law of Cosines"""
    numerator = a**2 + b**2 - c**2
    denominator = 2 * a * b
    cos_C = min(1, max(-1, numerator / denominator))  # Prevent domain errors
    return math.degrees(math.acos(cos_C))

def calculate_distance(x1, y1):
    """Calculate Euclidean distance from (0,0)"""
    return math.sqrt(x1**2 + y1**2)

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

def main():
    """Main function to run the Arducam depth camera processing"""
    global filtered_depthVal

    # Initialize camera
    cam = ac.ArducamCamera()
    ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("Failed to open camera. Error code:", ret)
        return

    ret = cam.start(ac.FrameType.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        return

    r = cam.getControl(ac.Control.RANGE)
    info = cam.getCameraInfo()
    print(f"Camera resolution: {info.width}x{info.height}")

    # Create windows
    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("preview", on_mouse)

    if info.device_type == ac.DeviceType.VGA:
        cv2.createTrackbar("confidence", "preview", confidence_value, 255, on_confidence_changed)

    while True:
        frame = cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.DepthData):
            # Retrieve depth and confidence data
            depth_buf = frame.depth_data()
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

            # Compute servo angles
            angleval = solve_angle_c(130, 130, filtered_depthVal)

            # Set servo positions based on filtered depth
            set_servo_angle(5, 0)
            set_servo_angle(4, 180)
            set_servo_angle(10, 180)
            set_servo_angle(11, 0)
            set_servo_angle(2, (90 + ((180 - angleval) / 2)))
            set_servo_angle(3, angleval)

            # Show preview
            cv2.imshow("preview", cv2.rotate(result_image, cv2.ROTATE_180))
            cam.releaseFrame(frame)

        # Exit condition
        key = cv2.waitKey(1)
        if key == ord("q"):
            break

    cam.stop()
    cam.close()

if __name__ == "__main__":
    main()
