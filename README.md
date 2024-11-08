# EE2028-Stm32L475
#STM32 Ghost Detection Project
This repository contains the firmware for a project focused on using the STM32 microcontroller for ghost detection and characterization. Developed as part of the EE2028 - Microcontroller Interfacing and Programming module in AY2023/2024, the project incorporates multiple sensors and unique features to create an interactive ghost-hunting experience.

#Core Features
These are the essential implementations required for all students in the module:

#Double Push Button Detection: Using interrupts to detect two quick button presses.
UART Telemetry: Sends telemetry data to a computer via UART for real-time monitoring.
Sensor Data Integration: Reads data from onboard sensors, including a magnetometer, IMU, and environmental sensors (pressure and temperature).

#Additional Features
6-axis IMU 3D Visualization: Uses sensor fusion to display real-time orientation, providing immersive visualization for detecting ghost direction.
Buzzer: Emits loud, high-frequency alerts when unusual readings are detected, working in sync with an LED for visual and audio cues.
Infrared Reflective Sensor: Detects close objects to trigger a self-destruct sequence with engaging ASCII visuals, enhancing user experience.
Ghost Characterization: Differentiates ghost types (Phantom, Spectre, Poltergeist) based on specific sensor readings, adding depth to detection.
Direction Detection: Provides both horizontal (cardinal) and vertical (above/below) directional cues, making the system more intuitive and realistic.
Key Learning Points
Through this project, I gained hands-on experience with real-time data handling, sensor fusion, and UART communication. Additionally, I learned how to implement device interrupts and developed practical problem-solving skills when dealing with sensor calibration and noise.

