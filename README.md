# Terminator - WRO 2025 Future Engineers

![Team Photo](./media/team_photo.jpg)
**Country:** Iran

---

## 1. Introduction
This repository contains the full documentation and source code for Team Terminator's entry into the World Robot Olympiad 2025 Future Engineers challenge.

Our core design philosophy is built on a **dual-controller architecture** that leverages the specialized strengths of two distinct processors. For high-level perception, we are utilizing an **NVIDIA Jetson Nano** for its powerful, dedicated GPU, which is ideal for the real-time image processing tasks required in the Obstacle Challenge. For low-level control, we have paired it with a **LEGO MINDSTORMS EV3** brick. The EV3 was chosen for its precise and reliable motor control and our team's familiarity with its straightforward sensor interface. This powerful architecture is bridged by a direct **serial connection**, allowing the Jetson to act as the "eyes" of the robot while the EV3 acts as the "muscles," creating a robust and responsive system.

### 2.1. Mobility Management

Our vehicle's mobility is based on a **rear-wheel-drive** system, built upon a custom-designed **LEGO Technic chassis**. This provides a modular and robust platform that allows for secure mounting of all electronic and mechanical components while adhering to the competition's four-wheel vehicle regulations.

* **Drivetrain:** A single **LEGO MINDSTORMS Large Motor** provides the driving force. It is directly connected to the rear axle, responsible for all forward and backward movements of the vehicle. This simple and direct connection ensures efficient power transfer and reliable performance.

* **Steering Mechanism:** Steering is controlled by a **LEGO MINDSTORMS Medium Motor**. To enhance control, we have implemented a **3:1 gear ratio** by connecting a 12-tooth gear on the motor to a 36-tooth gear on the steering axle. This was a deliberate design choice to **increase the steering torque**, allowing the robot to make more powerful and decisive turns. Additionally, this gearing expands the motor's effective rotational range, giving us finer control over the steering angle for improved stability and precision.

### 2.2. Power and Sense Management

Our robot’s electronic system is a **dual-controller architecture** designed for parallel processing, separating high-level vision tasks from low-level real-time control.

* **Controllers & Communication:** A **NVIDIA Jetson Nano** (running Ubuntu) serves as the primary visual processor. It communicates with a **LEGO MINDSTORMS EV3** brick (running ev3dev) via a **USB to TTL serial link**. The Jetson performs all image analysis and sends navigation commands to the EV3, which handles all motor control and feedback from its attached sensors.

* **Sensory System:**
    * **Vision:** A **Raspberry Pi Camera v2** is connected to the Jetson Nano, providing a continuous, high-resolution view of the field.
    * **Low-Level Sensing:** The EV3 is equipped with:
        * A **Color Sensor** (Port 1) to detect colored lines on the mat for initiating turns.
        * A **Front Ultrasonic Sensor** (Port 2) for forward-facing obstacle detection.
        * **Left and Right Ultrasonic Sensors** (Ports 3 & 4) for wall-following, lane centering, and safety overrides.

* **Power System:** The power system is carefully segregated to ensure stability.
    * The EV3 brick and its attached components are powered by its **standard LEGO rechargeable battery pack**.
    * The power-intensive Jetson Nano is powered by a **Waveshare UPS Module (3S)**. This module is equipped with three 18650 Li-ion batteries and provides a stable **5V / 5A output**, which is essential for the Jetson's peak performance. Its uninterruptible power supply (UPS) feature allows for simultaneous charging and operation, and its integrated protection circuits (against over-charge, over-current, and short-circuits) ensure the safety and longevity of our core processor.

## 3. Obstacle Management & Strategy

Our approach to navigation is tailored to the specific demands of each challenge, prioritizing simplicity where effective and employing a more complex, cooperative strategy when necessary.

### 3.1. Open Challenge Strategy

Our strategy for the Open Challenge prioritizes simplicity and reliability by relying exclusively on the EV3 and its attached sensors. As this challenge does not involve complex object recognition, the use of the Jetson Nano was deemed unnecessary. The logic is a robust and efficient finite-state machine:

1.  **Initial Direction Finding:** Upon starting, the robot drives forward until its **Color Sensor** detects a blue or yellow line. This detection sets the robot's turning direction (e.g., right for blue, left for yellow) for the remainder of the run.
2.  **Wall-Following:** Once a direction is established, the robot uses its **side ultrasonic sensor** and a Proportional (P) controller to maintain a consistent distance from the outer wall, ensuring it stays centered in the lane.
3.  **Corner Turning:** The **front ultrasonic sensor** continuously scans for the wall ahead. When a wall is detected within a pre-defined threshold, it signals a corner. The robot then executes a pre-programmed turn maneuver in the direction determined in the first step.

### 3.2. Obstacle Challenge Strategy

For the more complex Obstacle Challenge, we employ a hierarchical, **dual-controller strategy** where the Jetson Nano acts as the "vision system" and the EV3 acts as the "reflex and safety system."

**A. The Jetson Nano's Role: Visual Navigation**

The Jetson is responsible for all high-level perception and decision-making based on the camera feed.

1.  **Image Segmentation:** It processes each frame to segment the image into red, green, and black components using HSV color filtering.
2.  **Steering Calculation:** A steering command is calculated as a composite value based on the **position and area** of detected objects. The logic aims to steer away from black wall segments while simultaneously navigating around red and green obstacles according to the rules. The area of an object is used as a gain factor, meaning larger or closer objects trigger a stronger steering response.
3.  **Command Transmission:** This final steering value, along with a flag indicating if an object is currently visible, is transmitted to the EV3 via serial for execution.

**B. The EV3's Role: Execution and Safety**

The EV3 follows the Jetson's commands but has its own built-in overrides for precision and safety.

1.  **High-Priority "Locked Turn":** The EV3 is programmed with a "Locked Turn" maneuver. When its local **Color Sensor** detects a blue or yellow line at a corner, it **overrides any command from the Jetson** and executes a precise, pre-programmed turn. This was a deliberate design choice to ensure consistent and reliable navigation of the field's sharp corners, where visual data can be ambiguous or delayed.
2.  **Side Sensor Safety Net:** In all other states, while primarily following the Jetson's commands, the EV3 uses its **side ultrasonic sensors** as a constant safety net. If the robot strays too close to a side wall or a lateral object, the EV3 applies an immediate corrective steering action. This low-level reflex system prevents collisions faster than the vision-loop could react.

This layered strategy allows our robot to combine the advanced perception of the Jetson with the fast, reliable reflexes of the EV3, creating a system that is both intelligent and robust.

## 4. Software

Our robot's intelligence is driven by a distributed software architecture, with distinct roles assigned to the Jetson Nano and the EV3 to maximize performance and reliability. All programming was done in Python 3.

### 4.1. Code Description

The software architecture is built around a **master-slave communication model**.

* **Jetson Nano (The Master/Brain):** The Jetson runs a sophisticated Python script that leverages the **OpenCV** library for all computer vision tasks. To ensure a high, stable framerate without interrupting calculations, camera frames are captured in a separate, non-blocking thread using Python's `threading` library. The script's main loop processes these frames to detect walls and obstacles, calculates a final steering command, and transmits it to the EV3.

* **EV3 (The Slave/Muscles):** The EV3 runs a dedicated Python script using the **`ev3dev2` library**, which provides a robust interface for controlling motors and reading sensors. This script acts as a real-time execution layer. Its primary role is to listen for and execute steering commands from the Jetson, but it also contains critical, independent logic for safety (side-sensor collision avoidance) and precision maneuvers ("Locked Turns" at corners), making it an intelligent slave rather than a simple one.

* **Communication Protocol:** The two systems communicate over a serial link using a custom, lightweight string protocol formatted as `"Direction:Angle:ObjectSeen"`. This ensures low-latency and reliable transmission of navigation commands.

### 4.2. Code Files

The source code for this project is organized in the `src/` directory.

* `open_challenge.py`: A standalone script designed to run exclusively on the EV3 for the Open Challenge. It uses `ev3dev2` for direct sensor-based navigation.
* `jetson_controller.py`: The main vision and strategy script for the Obstacle Challenge, designed to run on the Jetson Nano.
* `obstacle_challenge_ev3.py`: The low-level control, safety, and command execution script for the Obstacle Challenge, designed to run on the EV3.

## 5. Vehicle Photos
* **Team Photo:** [Link to Team Photo](./t-photos/team.jpg)
* **Top View:** [Link to Top View Photo](./v-photos/top.jpg)
* **Bottom View:** [Link to Bottom View Photo](./v-photos/bottom.jpg)
* **Front View:** [Link to Front View Photo](./v-photos/front.jpg)
* **Rear View:** [Link to Rear View Photo](./v-photos/back.jpg)
* **Left Side View:** [Link to Left Side Photo](./v-photos/left.jpg)
* **Right Side View:** [Link to Right Side Photo](./v-photos/right.jpg)

## 6. Performance Videos
* **Open Challenge Video:** [https://www.youtube.com/watch?v=dQw4w9WgXcQ](http://googleusercontent.com/youtube.com/6)
* **Obstacle Challenge Video:** [https://www.youtube.com/watch?v=o-YBDTqX_ZU](http://googleusercontent.com/youtube.com/7)