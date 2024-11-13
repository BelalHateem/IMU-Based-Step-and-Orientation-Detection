# IMU-Based Step and Orientation Detection

## Overview

This project implements a step detection and heading calculation system using the **LSM303AGR** Inertial Measurement Unit (IMU) on an **STMicroelectronics SensorTile** device. The system captures accelerometer and magnetometer data to track user steps and orientation, displaying the results in a mobile app. The primary goal is to provide real-time step and heading data for pedestrian tracking, particularly for applications in personal safety.

## Project Structure

The project consists of:

- **Step Detection Algorithm**: Uses accelerometer data from the z-axis to detect steps based on thresholding.
- **Heading Calculation**: Uses magnetometer data to determine the user's heading, which is calculated relative to the device's startup orientation.
- **Mobile App Interface**: Displays step count and heading in real-time, leveraging the SensorTile’s Bluetooth capabilities.

## Key Components and Functionality

### Step Detection

Step detection is achieved by monitoring the z-axis of the accelerometer and using a threshold-based approach:

- **Moving Average Filter**: Averages the accelerometer data over a window of 15 samples (1.5 seconds) to reduce noise.
- **Threshold-Based Detection**: Detects steps when the averaged signal crosses predefined lower and upper thresholds.
  
This approach ensures accurate detection for walking, but performance may decline with changes in device orientation or faster movement (e.g., running).

### Heading Calculation

The heading is calculated using data from the magnetometer:

- **Magnetometer Initialization**: Configures the magnetometer to operate in high-resolution continuous mode.
- **Soft-Iron Calibration**: Adjusts x and y readings based on calibration data to correct for distortions.
- **Angle Calculation**: Uses arctangent calculations to determine the heading relative to the device's starting position.
  
The heading calculation provides reliable orientation data but can be affected by nearby magnetic interference due to the absence of hard-iron calibration.

## Algorithms and Methodology

1. **Magnetometer Calibration**: The system applies a soft-iron calibration to the magnetometer to offset distortions. Hard-iron calibration was not implemented, which could improve accuracy in magnetically noisy environments.
2. **High-Pass Filter**: Removes the gravitational component from accelerometer data, leaving only movement-related signals for accurate step detection.
3. **Threshold-Based Step Detection**: Identifies steps by monitoring when the accelerometer data crosses upper and lower thresholds after noise reduction.

## Limitations

- **Orientation Sensitivity**: The system requires the device to be held upright for accurate step detection. Steps are not reliably detected if the device is in a pocket or held at a different angle.
- **Static Thresholds**: Threshold values are hardcoded for walking. Faster movements, like running, are not consistently detected. Adaptive thresholding could improve performance across various speeds.
- **Magnetic Interference**: Without hard-iron calibration, the magnetometer's heading calculations are prone to distortion from nearby magnetic objects, limiting accuracy in some environments.

## Performance Analysis

- **Step Detection Accuracy**: Reliable for walking when held upright, but orientation changes can result in missed steps.
- **Heading Accuracy**: Effective in providing relative orientation but vulnerable to magnetic interference.
- **Real-Time Responsiveness**: The system updates every 100ms, providing responsive feedback with minimal latency. However, faster movements may experience slight delays due to the moving average filter.

## Future Improvements

- **Adaptive Thresholding**: Dynamically adjusting thresholds based on user movement could improve detection accuracy across different activities.
- **Hard-Iron Calibration**: Adding hard-iron calibration would reduce magnetic interference and improve heading accuracy.
- **Orientation Flexibility**: Expanding the detection algorithm to account for varied device orientations would enhance usability.

## Requirements

- **STMicroelectronics SensorTile** development kit with **LSM303AGR IMU**
- **Mobile App** to display step and heading data via Bluetooth

