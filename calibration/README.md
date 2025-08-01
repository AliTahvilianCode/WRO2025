# Calibration & Visualization Tools

This directory contains utility scripts and Jupyter Notebooks used for calibrating sensors, tuning parameters, and debugging the robot's vision system. These scripts are not part of the final competition code that runs on the robot but are essential tools for our development process.

## mask_visualization.ipynb

This Jupyter Notebook provides a live, 2x2 grid view from the robot's camera to assist with tuning HSV color ranges.

### Features
- **Live Camera Feed:** Shows the raw output from the Jetson Nano's camera.
- **Real-time Color Masks:** Displays the binary masks for Red, Green, and Black colors simultaneously.
- **Combined View:** Stitches the raw feed and the three masks into a single 2x2 window for easy comparison and tuning.

### How to Use
1.  Ensure you are running a Jupyter environment on the Jetson Nano.
2.  Navigate to this `calibration` directory in the terminal.
3.  Launch the notebook server by running the command: `jupyter notebook`.
4.  Open the `mask_visualization.ipynb` file in your web browser.
5.  Run all the cells in the notebook to start the live video stream.
6.  To stop the script, select "Interrupt" from the "Kernel" menu in Jupyter.

### Dependencies
- `opencv-python`
- `numpy`
- `ipywidgets` (for displaying the video feed in the notebook)