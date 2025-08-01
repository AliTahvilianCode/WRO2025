#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# WRO 2025 Future Engineers - Open Challenge (Version with 15-sample Median Filter)
# This script enables the robot to navigate a course by following walls,
# making turns at corners, and completing a set number of laps.
#

import time
import sys
from collections import deque
import statistics
from ev3dev2.motor import LargeMotor, MediumMotor, SpeedPercent, OUTPUT_B, OUTPUT_C
from ev3dev2.sensor.lego import UltrasonicSensor, ColorSensor
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4

# ==============================================================================
# 1. Hardware Initialization
# Connects the script to the physical motors and sensors.
# ==============================================================================
drive_motor = LargeMotor(OUTPUT_C)      # Motor for driving forward/backward.
steering_motor = MediumMotor(OUTPUT_B)  # Motor for steering.
color_sensor = ColorSensor(INPUT_1)     # Sensor to detect the start line color.
us_front = UltrasonicSensor(INPUT_2)    # Front sensor to detect turns.
us_left = UltrasonicSensor(INPUT_3)     # Left sensor for wall-following.
us_right = UltrasonicSensor(INPUT_4)    # Right sensor for wall-following.

# ==============================================================================
# 2. Reset Steering Motor
# Calibrates the steering to a reliable zero position at the start.
# This ensures that "0 degrees" is straight ahead.
# ==============================================================================
print("Resetting steering motor...")
steering_motor.reset()

# ==============================================================================
# 3. Program Constants
# These values define the robot's behavior and can be tuned for performance.
# ==============================================================================
KP = 4                          # Proportional gain for the wall-following controller. Higher values result in sharper reactions.
DRIVE_SPEED = -100              # Main driving speed (negative value moves the robot forward).
STEERING_SPEED = 40             # Speed of steering adjustments.
STOP_DISTANCE_CM = 50           # Distance (in cm) at which the front sensor detects a wall for a turn.
TURN_ANGLE = 90                 # Steering angle for making a 90-degree turn.
TURN_DRIVE_DEGREES = 1275       # How many degrees the drive motor turns during the turn maneuver.
TURN_COOLDOWN_DEGREES = 2000    # Motor degrees to travel after a turn before another turn can be initiated. Prevents multiple turns in one corner.
LAPS_TO_COMPLETE = 4            # The mission is complete after this many turns.
FINAL_MOVE_DEGREES = 2000       # Distance (in motor degrees) for the final run after completing all laps.

# ==============================================================================
# 4. Control Variables and Sensor Filters
# Initializes variables for state management and filters for sensor data stabilization.
# ==============================================================================
follow_side = None # Stores which wall to follow ('left' or 'right'). Starts as None.
laps_completed = 0 # Counter for completed turns.
turn_cooldown_position = 1 # Stores motor position after a turn to enforce the cooldown period.

# Deques are used as efficient fixed-size lists to store the last N sensor readings for median filtering.
# This helps to eliminate outlier noise from the ultrasonic sensors.
front_us_readings = deque(maxlen=15) # A larger filter for the critical front sensor to ensure turn reliability.
left_us_readings = deque(maxlen=5)
right_us_readings = deque(maxlen=5)

# Pre-fill the deques with initial readings to avoid errors at the start of the program.
print("Calibrating sensors by pre-filling filters...")
time.sleep(0.5) 
initial_front = us_front.distance_centimeters
initial_left = us_left.distance_centimeters
initial_right = us_right.distance_centimeters

for _ in range(15):
    front_us_readings.append(initial_front)

for _ in range(5):
    left_us_readings.append(initial_left)
    right_us_readings.append(initial_right)

print("Starting robot in 1 second.")
time.sleep(1)

try:
    # Start the robot moving forward at the defined speed.
    drive_motor.on(SpeedPercent(DRIVE_SPEED))

    # ==============================================================================
    # --- Main Loop ---
    # The core logic of the robot runs inside this loop until all laps are completed.
    # ==============================================================================
    while laps_completed < LAPS_TO_COMPLETE:
        # --- Sensor Value Filtering ---
        # Append new raw readings to the deques.
        front_us_readings.append(us_front.distance_centimeters)
        left_us_readings.append(us_left.distance_centimeters)
        right_us_readings.append(us_right.distance_centimeters)

        # Calculate the median of the stored readings to get a stable, noise-free value.
        front_us = statistics.median(front_us_readings)
        left_us = statistics.median(left_us_readings)
        right_us = statistics.median(right_us_readings)
        
        # --- Logging Section (for debugging) ---
        # Prints sensor data and robot state to the console.
        print("US -> F: {0:<5.1f} | L: {1:<5.1f} | R: {2:<5.1f} || Side: {3:<5} | Laps: {4}".format(
            front_us,
            left_us,
            right_us,
            str(follow_side),
            laps_completed
        ))
        
        # --- Motion Logic ---
        # This is a simple state machine. The robot is either looking for the start line or wall-following.
        if follow_side is None:
            # State 1: Before the first color is detected, drive straight.
            steering_motor.on_to_position(SpeedPercent(STEERING_SPEED), 0, block=False)
            
            # Check for the start line color to decide which wall to follow.
            detected_color = color_sensor.color_name
            if detected_color == 'Blue':
                follow_side = 'right' # If Blue is detected, follow the right wall.
                print("--- COLOR DETECTED: Blue -> Following RIGHT wall ---")
                continue # Skip the rest of the loop and start the next iteration.
            elif detected_color == 'Yellow':
                follow_side = 'left' # If Yellow is detected, follow the left wall.
                print("--- COLOR DETECTED: Yellow -> Following LEFT wall ---")
                continue
        else:
            # State 2: After a color is detected, use wall-following and turning logic.
            current_drive_pos = drive_motor.position

            # --- Turning Logic ---
            # Check if a wall is detected by the front sensor AND the cooldown period has passed.
            if front_us < STOP_DISTANCE_CM and current_drive_pos < turn_cooldown_position:
                laps_completed += 1 # Increment turn counter.

                # Determine turn direction based on which wall is being followed.
                steer_angle = -TURN_ANGLE if follow_side == 'right' else TURN_ANGLE
                
                # Execute the turn maneuver: steer and drive forward for a fixed distance.
                steering_motor.on_to_position(SpeedPercent(STEERING_SPEED), steer_angle, block=False)
                drive_motor.on_for_degrees(SpeedPercent(DRIVE_SPEED), TURN_DRIVE_DEGREES, brake=False, block=True)
                
                # After the turn, re-calibrate the target distance for the P-controller to the new wall distance.
                # This makes the wall-following adaptive to different corridor widths.
                if follow_side == 'right':
                    TARGET_DISTANCE_CM = us_right.distance_centimeters
                else:
                    TARGET_DISTANCE_CM = us_left.distance_centimeters
                
                # Set the cooldown position to prevent immediate re-triggering of another turn.
                turn_cooldown_position = drive_motor.position - TURN_COOLDOWN_DEGREES
                
                # Restart the main drive motor for continuous driving.
                drive_motor.on(SpeedPercent(DRIVE_SPEED))
                continue # Skip to the next loop iteration.

            if laps_completed == 0:
                # This ensures the P-controller does not run before the first turn is complete,
                # as TARGET_DISTANCE_CM is not yet defined.
                continue

            # --- Proportional Controller for Wall-Following ---
            # Calculates the error between the current distance and the target distance.
            if follow_side == 'right':
                error = right_us - TARGET_DISTANCE_CM
                target_angle = KP * error
            else: # follow_side == 'left'
                error = TARGET_DISTANCE_CM - left_us
                target_angle = KP * error

            # Clamp the steering angle to a reasonable range to prevent over-steering and motor strain.
            if target_angle > 100: target_angle = 100
            elif target_angle < -100: target_angle = -100
            
            # Apply the calculated steering adjustment.
            steering_motor.on_to_position(SpeedPercent(STEERING_SPEED), int(target_angle), block=False)

        time.sleep(0.01) # A small delay to ensure loop stability and prevent overwhelming the CPU.

    # ==============================================================================
    # --- Final Run with Wall-Following ---
    # After all laps (turns) are completed, the robot drives a final distance while
    # continuing to follow the wall.
    # ==============================================================================
    print("All laps completed. Executing final wall-following run.")
    final_move_start_pos = drive_motor.position

    # This loop runs the final stretch using the same wall-following logic as the main loop.
    while abs(drive_motor.position - final_move_start_pos) < FINAL_MOVE_DEGREES:
        # Filter sensor values continuously for a smooth final run.
        left_us_readings.append(us_left.distance_centimeters)
        right_us_readings.append(us_right.distance_centimeters)
        left_us = statistics.median(left_us_readings)
        right_us = statistics.median(right_us_readings)

        # Apply the proportional controller logic.
        if follow_side == 'right':
            error = right_us - TARGET_DISTANCE_CM
            target_angle = KP * error
        else: # follow_side == 'left'
            error = TARGET_DISTANCE_CM - left_us
            target_angle = KP * error
        
        # Clamp and apply steering angle.
        if target_angle > 100: target_angle = 100
        elif target_angle < -100: target_angle = -100
        steering_motor.on_to_position(SpeedPercent(STEERING_SPEED), int(target_angle), block=False)

        time.sleep(0.01)

finally:
    # This 'finally' block executes no matter how the program exits (normally or with an error).
    # It's a safety feature to ensure the robot stops.
    print("Mission finished. Stopping motors.")
    drive_motor.off(brake=True)
    steering_motor.off(brake=False)