# IOT-Robot-Chaser

## Project Overview

This project, 236333 - IoT Robot Chaser, focuses on developing a robot that can chase a moving target using IoT technologies.

## Progress

### Unit Tests

- **Servo Motor:** Input an angle via the serial monitor and verify correct rotation.
- **L298N (Rear TT Motors):** Input speed and direction for each motor via the serial monitor and verify correctness.

### Robot Components

- **WiFi Connection:**
  - The robot tries to connect to a stored nearby WiFi network.
  - If no network is found, it creates an AP "ESP32_AP" with an HTTP page that includes a WiFi manager interface.
  - The interface allows users to enter WiFi credentials, view hardware details, and update firmware.
  - Upon saving the credentials, they are stored in the ESP32’s NVS, and the robot attempts to connect to the network.

- **HTTP Server:**
  - After connecting to WiFi, the robot creates an HTTP server.
  - The server accepts commands for rear TT motors to drive at a certain speed and direction ('B' or 'F').
  - It also accepts commands for the servo motor angle, which controls the steering of the front wheels.

### Lab Components

1. **Data Reception and Conversion:**
   - A script receives the robot's and the target's data (coordinates and rotation) from the Motive/OptiTrack system.
   - Converts angles from quaternions to Euler angles.
   - Converts the axis system to Z-up to match the initial orientation of the robot and target.

2. **HTTP Command Script:**
   - A script to send HTTP commands to the robot.

3. **Pure Pursuit Algorithm:**
   - Implements the pure pursuit algorithm for tracking a moving target.
   - Includes mechanisms for smoother tracking and a simulation to visualize and refine the robot's movement logic.

4. **Integration:**
   - Combined the scripts to receive coordinates from the OptiTrack system, calculate the robot's motion using the pure pursuit algorithm, and send motion requests to the robot.

## Edge Cases

- The robot attempts to reconnect to WiFi whenever the connection is lost.
- During reconnection attempts, the LED blinks, and the robot does not move.

## Current Issues

(List any known issues or problems here)

## TODO

- **Testing and Adjustment:**
  - Observe the robot's actual motion and make necessary adjustments or add mechanisms for improvement.
  - Multi target tracking.
  - BMI160 sensor.