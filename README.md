# Microcontroller Programming and Interfacing-Stm32L475E-IOT01A


## STM32 Ghost Detection Project
This repository contains the firmware for a project focused on using the STM32 microcontroller for ghost detection and characterization. Developed as part of the EE2028 - Microcontroller Interfacing and Programming module in AY2024/2025, the project incorporates multiple sensors and unique features to create an interactive ghost-hunting experience.

## Core Features
**Double Push Button Detection**: Uses interrupts to detect two quick button presses, allowing smooth switching between Normal Mode and Ghost Busting Mode. A single press is also used for specific actions, such as capturing ghosts in Ghost Busting Mode.

**UART Telemetry**: Sends real-time telemetry data to a connected computer via UART, allowing continuous monitoring of environmental and sensor data. Warning messages are also transmitted when specific sensor thresholds are exceeded. This includes detailed information for each reading (temperature, pressure, humidity, accelerometer, gyroscope, and magnetometer), making it easy to analyze data in real time.

**Sensor Data Integration**: Collects data from a range of onboard sensors for a comprehensive environmental overview. Sensors include:

-Magnetometer: Functions as a spectral compass, detecting magnetic orientation to determine ghost proximity and direction.
-Temperature Sensor: Detects sudden temperature drops, which may indicate ghostly presence.
-Humidity Sensor: Tracks atmospheric humidity changes that could suggest paranormal activity.
-Pressure Sensor: Monitors changes in atmospheric pressure to detect potential ghost disturbances.
-Accelerometer and Gyroscope (IMU): Ensures the device remains steady and detects unusual movements or orientation shifts, which may signify ghost interaction.
**Calibration and Thresholds**:

-Magnetometer and IMU Calibration: Each sensor has an initial calibration routine to eliminate offsets, ensuring accurate readings. This calibration enhances the precision of ghost detection by setting reliable baselines.
Threshold Detection: Dynamic thresholds for each sensor (e.g., temperature, pressure, humidity) trigger warnings upon crossing set limits. This enables prompt detection of ghostly disturbances, with messages sent over UART detailing the specific anomalies.
Mode-based LED and Buzzer Feedback:

**LED Indicator**: The LED blinks at varying frequencies to signify proximity to detected ghosts, providing visual feedback based on ghost distance.

## Additional Features
- **6-axis IMU 3D Visualization**: 
This feature leverages the STM32's IMU (Inertial Measurement Unit) to capture real-time data from the accelerometer and gyroscope sensors, which provide information about the device’s orientation. When the ghost capture mode is activated (Mode 2), the STM32 main code starts sending telemetry data, including pitch, roll, and yaw readings, through the UART interface to a connected Python application. The Python code then processes this data for visualization.

Key points of this enhancement:

Data Fusion for Smooth Orientation Tracking: The Python code uses sensor fusion to combine gyroscope and accelerometer readings, processed through a complementary filter, which smooths out the data and reduces noise. This allows for a more stable and realistic visualization of the board’s movement.

Real-Time 3D Orientation Model: The Python application, utilizing OpenGL and Pygame, generates a 3D model that updates its orientation based on real-time pitch, roll, and yaw data. The model reacts to movements of the STM32 board, allowing users to see how the ghost detection system might respond to physical orientation changes.

Enhanced Immersive Experience: By visualizing orientation data in a 3D environment, users gain a more intuitive sense of directionality, essential for tracking the "ghost's" origin. This also highlights how orientation angles (pitch, roll, and yaw) are common in fields like Autonomous Underwater Vehicles (AUVs) and aviation, where precise spatial awareness is crucial.

- **Infrared Reflective Sensor**: Detects close objects to trigger a self-destruct sequence with engaging ASCII visuals, enhancing user experience.
For the Infrared Reflective Sensor enhancement, the self-destruct sequence was designed to simulate a catastrophic event triggered when the ghost is extremely close to the device. By calibrating the infrared reflective sensor to detect objects within 4mm (the closest possible detection range), the system acts as though the ghost has breached the last line of defense, initiating an immediate self-destruct protocol.

Self-Destruct Sequence Breakdown:
Detection and Trigger: When the infrared sensor detects an object at the critical distance, it initiates a sequence designed to prevent further damage. This sequence serves as a safeguard, symbolizing the device's last-ditch effort against the "ghost."

ASCII Visuals for Immersion: The system uses custom ASCII art to convey urgency and severity. The sequence begins with an "ABORT" message in ASCII, followed by a dramatic countdown from 3 to 1 and a final explosion. This ASCII art was generated through text-to-ASCII conversion, providing a retro-themed, engaging visual for the user.

System Lockdown: The self-destruct sequence ends with the UART being disabled, cutting off communications as if the device has fully "shut down." This enhances the dramatic effect and the immersive experience for the user.

The entire sequence is pragmatic in enhancing the project's interactivity while showcasing creative coding techniques for visual and auditory feedback through UART and buzzer integration.

- **Buzzer**: Emits loud, high-frequency alerts when unusual readings are detected, working in sync with an LED for visual and audio cues.
For the Buzzer enhancement, the auditory alerts are designed to work alongside the LED for a synchronized, multi-sensory experience. This integration ensures that alerts are clear and adaptive to varying levels of detected activity, creating a dynamic interaction.

Startup Beep Sequence: The buzzer provides a quick, responsive startup beep when the device enters ghost-busting mode. The beep pattern is generated by rapid toggling, confirming mode activation and preparing users for "ghost detection" in real-time.

Self-Destruct Countdown Beeps: During the self-destruct sequence, the buzzer emits rapid pulses for each countdown number. This high-frequency beeping, synchronized with the ASCII countdown display, builds intensity, simulating a dramatic emergency shutdown. 
Coordination with LED Indicator: The buzzer beeps in tandem with LED2 at specific frequencies depending on magnetometer readings, providing an adaptive alert system based on the detected intensity:

2 Hz when magnetometer data (mag_data_check) is less than 300.
8 Hz for moderate levels.
16 Hz for high levels, intensifying the alert for stronger detections.
This coordination provides a nuanced feedback system, combining sound and light at variable intensities to match the alert level. 

- **Ghost Characterization**: Differentiates ghost types (Phantom, Spectre, Poltergeist) based on specific sensor readings, adding depth to detection.
I applied insights from the NUS module GEC1024 "Ghosts and Spirits in Society and Culture," where I learned about a ghost framework by anthropologists Waskul and Waskul. This framework characterizes ghostly entities based on distinct behaviors and attributes, allowing for a structured classification of ghost types, such as Phantoms, Spectres, and Poltergeists.

Conveniently, each ghost characteristic aligns well with specific sensor data. For example, temperature drops can indicate the presence of a Phantom, pressure anomalies can signal a Spectre, and changes in gyro and accelerometer readings suggest Poltergeist activity. By setting threshold values for each sensor, our system can effectively identify and respond to different ghost types, adding depth and functionality to the ghost detection system through this immersive
| Ghost Type   | Characteristics                   | Sensor Triggered        |
|--------------|-----------------------------------|--------------------------|
| **Phantom**  | Menacing, dream-like state        | Temperature sensor       |
| **Spectre**  | Menacing                          | Pressure sensor          |
| **Poltergeist** | Noisy ghosts, can manipulate physical environment | Gyro and Accelerometer |

This characterization allows us to map each ghost type to specific environmental changes detected by sensors. When certain sensor thresholds are crossed, the system identifies the corresponding ghost type and sends relevant alerts, enhancing the realism of the ghost-hunting experience.
- **Direction Detection**: Provides both horizontal (cardinal) and vertical (above/below) directional cues, making the system more intuitive and realistic.
For the direction detection of the ghost, we identified that our magnetometer’s zero-gauss accuracy is not high enough to accurately pinpoint the ghost’s exact 3D location. Typically, achieving this would require intense calibration, which wasn't feasible within our resource constraints.

To solve this, we developed a straightforward approach: isolating the detection to two planes. We focused on detecting the ghost’s position in the vertical plane (above or below) and the horizontal plane (cardinal directions: north, south, east, west). This way, our ghostbusters can intuitively know if the ghost is above or below, as well as the horizontal direction it’s coming from.

The code checks for threshold values in each axis (X, Y, Z) to determine the direction. If a reading crosses the threshold, it indicates a cardinal or vertical direction. This solution enhances user experience by giving a clear, simple directional cue without needing complex 3D calibration.

## Key Learning Points
Through this project, I gained hands-on experience with real-time data handling, sensor fusion, and UART communication. Additionally, I learned how to implement device interrupts and developed practical problem-solving skills when dealing with sensor calibration and noise.

## Key collaborators
3Dfusion.py GK
Extra (fluff) peripherals Yulyuri

