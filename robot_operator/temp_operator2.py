import numpy as np
import math
import time
import requests
import chaser_data_handling



# Define constants
k = 0.1  # Look ahead gain
Lfc = 2.0  # Look-ahead distance
Kp = 1.0  # Speed proportional gain
dt = 0.1  # Time tick
WB = 2.9  # Wheelbase of the vehicle

# Global state variables
robot_state = None
target_x = 0.0
target_y = 0.0
target_yaw = 0.0
last_pwm_speed = 0.0


# State class
class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += (self.v / WB) * math.tan(delta) * dt
        self.v += a * dt


# Convert speed to PWM
def speed_to_pwm(speed):
    # Simple mapping, adjust as needed
    return int(speed * 50)  # Example scaling factor


# Convert steering angle to PWM
def angle_to_pwm(angle):
    angle_deg = math.degrees(angle)
    return int(60 + (angle_deg / 60) * 60)  # Example mapping, adjust as needed


# Send motion command
def send_motion_command(ip_address, port, left_direction, left_pwm, right_direction, right_pwm, angle):
    left_pwm = max(0, min(255, left_pwm))
    right_pwm = max(0, min(255, right_pwm))
    angle = max(60, min(120, angle))
    left_pwm_str = f"{left_pwm:03d}"
    right_pwm_str = f"{right_pwm:03d}"
    angle_str = f"{angle:03d}"
    command = f"{left_direction}{left_pwm_str}{right_direction}{right_pwm_str}A{angle_str}A"
    url = f"http://{ip_address}:{port}/{command}"
    response = requests.get(url)
    return response


# Calculate control inputs
def calculate_control(state, target_x, target_y):
    dx = target_x - state.x
    dy = target_y - state.y
    target_heading = math.atan2(dy, dx)
    alpha = target_heading - state.yaw
    Lf = k * state.v + Lfc
    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    # Compute acceleration (e.g., proportional to the speed error)
    target_speed = 10.0 / 3.6  # Example target speed in m/s
    a = Kp * (target_speed - state.v)

    # Convert steering angle to PWM
    steering_pwm = angle_to_pwm(delta)

    return a, steering_pwm


# Update algorithm with real-time data
def update_algorithm():
    global robot_state, target_x, target_y, last_pwm_speed

    # Calculate control inputs
    ai, steering_pwm = calculate_control(robot_state, target_x, target_y)

    # Convert control inputs to PWM
    left_pwm = speed_to_pwm(robot_state.v)
    right_pwm = left_pwm  # Assuming symmetric PWM values for both motors

    # Send commands to robot
    left_direction = 'F'
    right_direction = 'F'
    send_motion_command("192.168.0.101", 80, left_direction, left_pwm, right_direction, right_pwm, steering_pwm)


# Receive new frame function
def receive_new_frame(data_frame):
    global robot_state, target_x, target_y, target_yaw, last_pwm_speed

    # Process each rigid body
    for ms in data_frame.rigid_bodies:
        if ms.id_num == 600:  # Chaser (robot)
            c_pos, c_rot, c_rad = chaser_data_handling.handle_frame(ms, "CHASER")
            # Update robot state
            robot_state.x = c_pos[0]
            robot_state.y = c_pos[1]
            robot_state.yaw = math.radians(c_rot[2])  # Convert yaw from degrees to radians
            robot_state.v = last_pwm_speed  # Update velocity with last known PWM speed

        if ms.id_num == 601:  # Target
            t_pos, t_rot, t_rad = chaser_data_handling.handle_frame(ms, "TARGET")
            target_x, target_y = t_pos[0], t_pos[1]
            target_yaw = t_rot[2]  # Assuming t_rot gives yaw in radians

    print("Fill the array for animation")
    print("Algorithm Goes here")

    # Update the robot state and control based on the latest data
    update_algorithm()

    print("-----------------------------------------------------------------")


if __name__ == '__main__':
    print("Robot operator start")
    ip_address = '192.168.0.101'
    port = 80
    robot_operator(ip_address, port)
