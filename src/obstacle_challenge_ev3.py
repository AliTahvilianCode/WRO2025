#!/usr/bin/env python3
# ev3_controller.py
# This script runs on the EV3 brick. It acts as a low-level controller,
# managing motors and side sensors, while receiving high-level navigation
# commands from a Jetson Nano via serial communication.

import serial
import time
from ev3dev2.motor import LargeMotor, MediumMotor, OUTPUT_B, OUTPUT_C, SpeedPercent
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_3, INPUT_4

# ==============================================================================
# 1. Configuration Constants
# ==============================================================================
SERIAL_PORT = '/dev/ttyUSB0'   # USB port for serial communication with Jetson.
BAUD_RATE = 9600               # Data rate for serial communication.
DRIVE_SPEED = -65              # Constant driving speed (negative for forward).
DRIVE_MOTOR_EXIT_ROTATION = 1400 # Motor degrees to drive during a locked turn before exiting.
LOCKED_TURN_ANGLE = 95         # Fixed steering angle for turns triggered by the color sensor.

# Cooldown period (in motor rotations) after a color sensor trigger to prevent immediate re-triggering.
COLOR_SENSOR_COOLDOWN_ROTATION = 2000 

# --- Side Sensor Safety Overrides ---
# These parameters create a safety buffer to prevent crashing into side walls.
SIDE_EMERGENCY_CM = 6          # If distance to a side wall is less than this, trigger an emergency turn.
SIDE_EMERGENCY_ANGLE = 50      # The angle for the emergency turn.
SIDE_WARN_1_CM = 14            # First warning distance threshold.
SIDE_WARN_1_ANGLE = 30         # Corrective angle for the first warning.
SIDE_WARN_2_CM = 18            # Second warning distance threshold.
SIDE_WARN_2_ANGLE = 15         # Corrective angle for the second warning.

# --- Self-Centering Logic ---
Kp_centering = 1.6             # Proportional gain for the self-centering logic (using side sensors).
OBJECT_SEEN_COOLDOWN_ROTATION = 800 # How long (in motor rotations) to ignore self-centering after seeing an object.
MAX_SELF_CENTERING_ANGLE = 40  # Maximum angle allowed for the self-centering correction.

# --- Locked Turn Extension ---
LOCKED_TURN_EXTENSION_ROTATION = 300 # Motor degrees to continue the turn if an object is seen during the turn.


# ==============================================================================
# 2. Hardware Initialization
# ==============================================================================
drive_motor = LargeMotor(OUTPUT_C)
steer_motor = MediumMotor(OUTPUT_B)
ultrasonic_left = UltrasonicSensor(INPUT_3)
ultrasonic_right = UltrasonicSensor(INPUT_4)
color_sensor = ColorSensor(INPUT_1)

# Reset steering motor to its zero position for calibration.
steer_motor.reset()

# Initialize serial communication.
ser = None
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except Exception:
    # If serial fails, stop everything and exit.
    drive_motor.off()
    steer_motor.off()
    exit()

# ==============================================================================
# 3. Main Control Loop
# ==============================================================================
try:
    # Start the robot moving forward.
    drive_motor.on(speed=SpeedPercent(DRIVE_SPEED))
    
    # --- State Variables ---
    direction, angle_from_jetson, object_seen = 'N', 0, False # Variables to store data from Jetson.
    
    # State flags for managing complex behaviors like turns.
    is_in_locked_turn = False
    locked_turn_angle = 0
    locked_turn_drive_motor_start_pos = 0
    
    is_color_trigger_on_cooldown = False
    color_trigger_cooldown_start_pos = 0
    
    last_object_seen_motor_pos = 0
    is_in_turn_extension = False
    turn_extension_start_pos = 0
    
    # Initialize sensor reading lists for median filtering.
    left_us_readings = [ultrasonic_left.distance_centimeters for _ in range(3)]
    right_us_readings = [ultrasonic_right.distance_centimeters for _ in range(3)]

    while True:
        # --- Sensor Data Filtering ---
        # Use a simple moving median filter of size 3 to stabilize ultrasonic readings.
        left_us_readings = left_us_readings[1:] + [ultrasonic_left.distance_centimeters]
        right_us_readings = right_us_readings[1:] + [ultrasonic_right.distance_centimeters]
        dist_left = sorted(left_us_readings)[1]
        dist_right = sorted(right_us_readings)[1]
        
        # Read the current color from the color sensor.
        detected_color = color_sensor.color_name

        # --- Serial Data Processing ---
        # Read the latest complete command from the Jetson.
        if ser.in_waiting > 0:
            last_line = ""
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line: last_line = line
            
            # Parse the three-part command: "Direction:Angle:ObjectSeen"
            if last_line:
                try:
                    parts = last_line.split(':')
                    if len(parts) == 3:
                        direction, angle_from_jetson, object_seen = parts[0], int(parts[1]), (int(parts[2])==1)
                except:
                    pass # Ignore malformed data.
        
        # Record the motor position when an object is last seen.
        if object_seen:
            last_object_seen_motor_pos = drive_motor.position

        # --- State Management Logic ---

        # 1. Manage color sensor cooldown period.
        if is_color_trigger_on_cooldown:
            if abs(drive_motor.position - color_trigger_cooldown_start_pos) >= COLOR_SENSOR_COOLDOWN_ROTATION:
                is_color_trigger_on_cooldown = False
        
        # 2. Manage the "Locked Turn" state.
        if is_in_locked_turn:
            # Handle the "Turn Extension" sub-state.
            if is_in_turn_extension:
                if abs(drive_motor.position - turn_extension_start_pos) >= LOCKED_TURN_EXTENSION_ROTATION:
                    is_in_locked_turn, is_in_turn_extension = False, False
            else:
                drive_motor_rotation = abs(drive_motor.position - locked_turn_drive_motor_start_pos)
                # Check for conditions to exit the locked turn.
                if object_seen:
                    # If an object is seen in the opposite direction of the turn, exit the turn.
                    if (locked_turn_angle * angle_from_jetson) < 0:
                        is_in_locked_turn = False
                    # Otherwise, extend the turn to avoid the object.
                    else:
                        is_in_turn_extension, turn_extension_start_pos = True, drive_motor.position
                # Exit the turn after driving a certain distance.
                elif drive_motor_rotation >= DRIVE_MOTOR_EXIT_ROTATION:
                    is_in_locked_turn = False
        
        # 3. Check for conditions to START a locked turn.
        # A turn starts if not already in one, the color cooldown is off, and a line is detected.
        if not is_in_locked_turn and not is_color_trigger_on_cooldown and (detected_color == 'Blue' or detected_color == 'Yellow'):
            is_in_locked_turn, locked_turn_drive_motor_start_pos = True, drive_motor.position
            # Turn left for Yellow, right for Blue.
            locked_turn_angle = -LOCKED_TURN_ANGLE if detected_color == 'Blue' else LOCKED_TURN_ANGLE
            # Activate the cooldown to prevent immediate re-triggering.
            is_color_trigger_on_cooldown, color_trigger_cooldown_start_pos = True, drive_motor.position
        
        if not is_in_locked_turn and ('reason' in locals() and "Locked Turn" in reason):
             is_in_turn_extension = False # Reset extension state after exiting a locked turn.

        # --- Steering Decision Tree ---
        # Determines the final steering angle based on a hierarchy of states.
        steer_angle = 0; reason = "Default"
        if is_in_locked_turn:
            # HIGHEST PRIORITY: If in a locked turn, use the fixed turn angle.
            steer_angle = locked_turn_angle
            reason = "Locked Turn Extension" if is_in_turn_extension else "Locked Turn"
        else:
            # When not in a locked turn, calculate steering based on sensors.
            steer_from_jetson = angle_from_jetson
            steer_from_side_sensors = 0
            side_warn_condition = dist_left < SIDE_WARN_2_CM or dist_right < SIDE_WARN_2_CM
            
            if side_warn_condition:
                # PRIORITY 2: Side wall proximity override.
                # If too close to a wall, apply a corrective angle.
                closer_side_dist, is_left_closer = (dist_left, True) if dist_left < dist_right else (dist_right, False)
                turn_angle = 0
                if closer_side_dist < SIDE_EMERGENCY_CM: turn_angle = SIDE_EMERGENCY_ANGLE
                elif closer_side_dist < SIDE_WARN_1_CM: turn_angle = SIDE_WARN_1_ANGLE
                else: turn_angle = SIDE_WARN_2_ANGLE
                steer_from_side_sensors = turn_angle if is_left_closer else -turn_angle
                steer_angle = steer_from_jetson + steer_from_side_sensors
                reason = "Jetson + Side"
            elif not object_seen:
                # PRIORITY 3: Self-centering when no object is visible.
                rotation_since_last_object = abs(drive_motor.position - last_object_seen_motor_pos)
                if rotation_since_last_object < OBJECT_SEEN_COOLDOWN_ROTATION:
                    # Trust Jetson for a short period after an object disappears.
                    steer_angle, reason = steer_from_jetson, "Object Seen Cooldown"
                else:
                    # Use side sensors to center the robot in the lane.
                    side_error = dist_left - dist_right
                    steer_angle_from_centering = -(Kp_centering * side_error)
                    # Limit the self-centering correction angle.
                    steer_angle_from_centering = max(-MAX_SELF_CENTERING_ANGLE, min(MAX_SELF_CENTERING_ANGLE, steer_angle_from_centering))
                    steer_angle, reason = steer_from_jetson + steer_angle_from_centering, "Self-Centering"
            else:
                # DEFAULT: If an object is seen and walls are not too close, trust the Jetson.
                steer_angle, reason = steer_from_jetson, "Jetson Only"
        
        # --- Final Steering Execution ---
        # Limit the final angle to the motor's safe range and apply it.
        final_steer_angle = max(-95, min(95, steer_angle))
        steer_motor.on_to_position(SpeedPercent(100), int(final_steer_angle), block=False)
        
        time.sleep(0.02) # Short delay for loop stability.

except KeyboardInterrupt:
    # Allows the program to be stopped cleanly with Ctrl+C.
    pass
finally:
    # This block runs no matter how the program exits, ensuring a safe shutdown.
    drive_motor.off(brake=True)
    if ser: ser.close()
    # Center the steering before turning off the motor.
    steer_motor.on_to_position(SpeedPercent(75), 0)
    time.sleep(0.5)
    steer_motor.off(brake=False)