#!/usr/bin/env python3
# jetson_controller.py
# This script runs on the Jetson Nano. It acts as the high-level "brain",
# processing camera images to detect walls and colored obstacles. It then
# computes a steering command and sends it to the EV3 for execution.

import cv2
import numpy as np
import serial
import time
import threading
import atexit

# ==============================================================================
# 1. FastCamera Class
# A threaded class to read frames from the GStreamer pipeline without blocking
# the main processing loop, ensuring a high and stable frame rate.
# ==============================================================================
class FastCamera:
    def __init__(self, width=640, height=480, framerate=30, sensor_id=0):
        # GStreamer pipeline for efficient camera access on Jetson devices.
        self.gstreamer_pipeline = (f"nvarguscamerasrc sensor-id={sensor_id} ! "
                                   f"video/x-raw(memory:NVMM), width=(int)1280, height=(int)720, framerate=(fraction){framerate}/1 ! "
                                   f"nvvidconv flip-method=2 ! " # Flips the image vertically and horizontally.
                                   f"video/x-raw, width=(int){width}, height=(int){height}, format=(string)BGRx ! "
                                   f"videoconvert ! video/x-raw, format=(string)BGR ! "
                                   f"appsink drop=true")
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened(): raise RuntimeError("Camera GStreamer failed to open.")
        
        self.frame = np.zeros((height, width, 3), dtype=np.uint8)
        self.running = True
        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = True
        self.thread.start()
        atexit.register(self.stop) # Ensure camera is released on program exit.

    def _reader(self):
        # Internal method that runs in a separate thread to continuously read frames.
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                self.frame = frame
        self.cap.release()

    def read(self):
        # Returns the most recently captured frame.
        return self.frame

    def stop(self):
        # Stops the reading thread and releases the camera.
        self.running = False
        if self.thread.is_alive():
            self.thread.join()

# ==============================================================================
# 2. Utility Function
# ==============================================================================
def find_largest_contour(mask, min_area):
    """Finds and returns the largest contour in a binary mask if its area is above a minimum threshold."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest_contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest_contour) > min_area:
        return largest_contour
    return None

# ==============================================================================
# 3. Initialization and Parameters
# ==============================================================================
WIDTH, HEIGHT = 480, 270
camera = FastCamera(width=WIDTH, height=HEIGHT)

SERIAL_PORT = '/dev/ttyTHS1' # Jetson's hardware serial port.
BAUD_RATE = 9600
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print("Jetson: Serial port connected.")
except Exception as e:
    print(f"Jetson: Error connecting to serial port: {e}")

# --- Color Detection and Control Parameters ---
# HSV color ranges for segmentation.
lower_red1 = np.array([0, 100, 70]); upper_red1 = np.array([7, 255, 255])
lower_red2 = np.array([160, 65, 65]); upper_red2 = np.array([180, 255, 255]) # Red wraps around 180 in HSV.
lower_green = np.array([40, 75, 75]); upper_green = np.array([90, 255, 255])
lower_black = np.array([0, 0, 0]); upper_black = np.array([180, 255, 120])

# Proportional gains for the camera-based steering controller.
Kp_camera_error = 0.03   # Gain for the error in horizontal position.
Kp_camera_area = 0.002   # Gain related to the object's area (larger objects cause stronger turns).

# Minimum area (in pixels) for an object to be considered for turning.
MIN_AREA_FOR_TURN = 250

# Target horizontal positions for red and green objects.
# The robot will try to move red objects to the left edge and green objects to the right edge.
RED_TARGET_X = int(WIDTH * 0.05)
GREEN_TARGET_X = int(WIDTH * 0.95)

# Thresholds for wall detection based on the number of black pixels.
WALL_PIXEL_THRESHOLDS = [40000, 45000, 50000]
# Corresponding steering angles for each wall detection threshold.
WALL_STEER_ANGLES     = [15,    50,    70]
# Minimum number of black pixels required to trigger wall detection logic.
TOTAL_BLACK_PIXEL_THRESHOLD = 1000

print("Jetson: Starting main loop... Press Ctrl+C to stop.")

# ==============================================================================
# 4. Main Processing Loop
# ==============================================================================
try:
    kernel = np.ones((3, 3), np.uint8) # Kernel for morphological operations.
    while True:
        frame = camera.read()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # --- Image Segmentation ---
        # Create masks for each color.
        red_mask = cv2.add(cv2.inRange(hsv_frame, lower_red1, upper_red1), cv2.inRange(hsv_frame, lower_red2, upper_red2))
        green_mask = cv2.inRange(hsv_frame, lower_green, upper_green)
        black_mask = cv2.inRange(hsv_frame, lower_black, upper_black)
        
        # Refine the black mask by removing pixels that are also red or green.
        # This prevents colored obstacles from being mistaken as part of the black wall.
        temp_mask = cv2.subtract(black_mask, red_mask)
        final_black_mask = cv2.subtract(temp_mask, green_mask)
        # Apply morphological opening to remove small noise from the black mask.
        final_black_mask = cv2.morphologyEx(final_black_mask, cv2.MORPH_OPEN, kernel)
        
        # --- Steering Calculation ---
        # The final steering command is a sum of wall avoidance and object navigation.

        # 1. Calculate Wall Steering Value
        wall_steer_value = 0
        total_black_pixels = cv2.countNonZero(final_black_mask)
        if total_black_pixels > TOTAL_BLACK_PIXEL_THRESHOLD:
            mid_x = int(WIDTH / 2)
            left_pixels = cv2.countNonZero(final_black_mask[:, 0:mid_x])
            right_pixels = cv2.countNonZero(final_black_mask[:, mid_x:WIDTH])
            
            # Determine which side has more black pixels (wall is closer).
            pixel_magnitude, turn_sign = (left_pixels, 1) if left_pixels > right_pixels else (right_pixels, -1)
            
            # Determine the magnitude of the turn based on how many pixels are detected.
            turn_magnitude = 0
            if pixel_magnitude > WALL_PIXEL_THRESHOLDS[2]: turn_magnitude = WALL_STEER_ANGLES[2]
            elif pixel_magnitude > WALL_PIXEL_THRESHOLDS[1]: turn_magnitude = WALL_STEER_ANGLES[1]
            elif pixel_magnitude > WALL_PIXEL_THRESHOLDS[0]: turn_magnitude = WALL_STEER_ANGLES[0]
            
            wall_steer_value = turn_magnitude * turn_sign
        
        # 2. Calculate Colored Object Steering Value
        object_steer_value = 0
        object_seen_flag = False
        red_contour = find_largest_contour(red_mask, MIN_AREA_FOR_TURN)
        green_contour = find_largest_contour(green_mask, MIN_AREA_FOR_TURN)
        area_red = cv2.contourArea(red_contour) if red_contour is not None else 0
        area_green = cv2.contourArea(green_contour) if green_contour is not None else 0

        if area_green > 0 or area_red > 0:
            object_seen_flag = True
            # Determine which object is dominant (larger area).
            if area_green > area_red:
                # Steer to keep the GREEN object on the RIGHT side of the view.
                M = cv2.moments(green_contour)
                cx = int(M["m10"] / M["m00"])
                if cx < GREEN_TARGET_X:
                    error = GREEN_TARGET_X - cx
                    object_steer_value = -((Kp_camera_area*area_green)*(Kp_camera_error*error))-(Kp_camera_area*area_green)
            else:
                # Steer to keep the RED object on the LEFT side of the view.
                M = cv2.moments(red_contour)
                cx = int(M["m10"] / M["m00"])
                if cx > RED_TARGET_X:
                    error = cx - RED_TARGET_X
                    object_steer_value = ((Kp_camera_area*area_red)*(Kp_camera_error*error))+(Kp_camera_area*area_red)

        # --- Command Fusion and Sending ---
        # Combine wall and object steering, and limit the final value.
        final_steer_value = np.clip(wall_steer_value + object_steer_value, -90, 90)
        
        # Determine a simple direction character for the EV3.
        direction = 'D' # Default direction
        if final_steer_value > 5: direction = 'R'
        elif final_steer_value < -5: direction = 'L'
        
        # Format the command into the three-part protocol: "Direction:Angle:ObjectSeen"
        command = "{}:{}:{}".format(direction, int(final_steer_value), 1 if object_seen_flag else 0)
        
        # Send the command to the EV3.
        if ser:
            ser.write((command + '\n').encode('utf-8'))
            print("Jetson Sent: {}".format(command))
            
        time.sleep(0.05) # Loop delay.

except KeyboardInterrupt:
    print("\nLoop stopped by user.")
finally:
    # Safely stop the camera and serial communication on exit.
    camera.stop()
    if ser:
        ser.write(b'S:0:0\n') # Send a final stop command.
        ser.close()
    print("Jetson program stopped.")