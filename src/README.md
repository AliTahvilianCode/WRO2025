- **`open_challenge.py`**
  - A standalone script designed to run exclusively on the EV3 for the Open Challenge. It uses `ev3dev2` for direct sensor-based navigation.

- **`obstacle_challenge_jetson.py`**
  - The main vision and strategy script for the Obstacle Challenge, designed to run on the Jetson Nano. It uses OpenCV to process the camera feed and sends commands to the EV3.

- **`obstacle_challenge_ev3.py`**
  - The low-level control, safety, and command execution script for the Obstacle Challenge. It runs on the EV3 and acts as the "muscles" of the robot, responding to the Jetson's commands.