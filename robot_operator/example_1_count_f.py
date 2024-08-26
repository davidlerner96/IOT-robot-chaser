import math
import time
import socket
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

from natnet_client import DataDescriptions, DataFrame, NatNetClient

import requests

ip_address = socket.gethostbyname("esp32.local")
port = 80
print("Robot's ip address: " + ip_address)
print("Operator's ip address: " + socket.gethostbyname(socket.gethostname()))


def send_motion_command(ip_address, port, left_direction, left_pwm, right_direction, right_pwm, angle):
    """
    Sends a motion command to the robot car via HTTP GET request.

    Parameters:
    - ip_address (str): The IP address of the robot car's HTTP server.
    - port (int): The port number of the robot car's HTTP server.
    - left_direction (str): The direction of the left motor ('F' for forward, 'B' for backward).
    - left_pwm (int): The PWM speed of the left motor (0-255).
    - right_direction (str): The direction of the right motor ('F' for forward, 'B' for backward).
    - right_pwm (int): The PWM speed of the right motor (0-255).
    - angle (int): The angle of the front steering wheels (60-120).

    Returns:
    - response (requests.Response): The response object from the HTTP GET request.
    """
    # Ensure PWM values are within valid range (0-255)
    left_pwm = max(0, min(255, left_pwm))
    right_pwm = max(0, min(255, right_pwm))

    # Ensure angle is within valid range (60-120)
    angle = max(60, min(120, angle))

    # Format PWM values and angle to 3 digits
    left_pwm_str = f"{left_pwm:03d}"
    right_pwm_str = f"{right_pwm:03d}"
    angle_str = f"{angle:03d}"

    # Construct the command string
    command = f"{left_direction}{left_pwm_str}{right_direction}{right_pwm_str}A{angle_str}A"

    # Construct the URL
    url = f"http://{ip_address}:{port}/{command}"

    # Send the GET request
    response = requests.get(url)

    return response


########


def normalize_angle(angle):
    """Normalize the angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def distance(p1, p2):
    """Calculate the Euclidean distance between two points."""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def pure_pursuit(x_r, y_r, theta, target, lookahead_distance):
    """
    Pure Pursuit algorithm to calculate the steering angle to follow a path.

    Parameters:
    x_r, y_r: Coordinates of the robot's rear axle midpoint
    theta: Orientation of the robot
    target: Current target point (x, y)
    lookahead_distance: Lookahead distance

    Returns:
    Steering angle to follow the path.
    """

    # Predict the lookahead point based on the target's location and velocity
    lookahead_point = (target[0] + lookahead_distance * math.cos(target[2]),
                       target[1] + lookahead_distance * math.sin(target[2]))

    # Calculate the target angle (alpha)
    alpha = math.atan2(lookahead_point[1] - y_r, lookahead_point[0] - x_r)

    # Calculate the angle difference (beta)
    beta = normalize_angle(alpha - theta)

    # Calculate the steering angle
    steering_angle = math.atan2(2 * lookahead_distance * math.sin(beta), lookahead_distance)

    # Constrain the steering angle to be within the limits of the robot
    max_steering_angle = math.radians(30)  # 30 degrees
    steering_angle = max(min(steering_angle, max_steering_angle), -max_steering_angle)

    return steering_angle, lookahead_point


def kinematic_bicycle_model(x, y, theta, v, delta, L, dt):
    """
    Kinematic bicycle model to update the robot's state.

    Parameters:
    x, y: Current position of the robot's rear axle midpoint
    theta: Current orientation of the robot
    v: Current velocity of the robot
    delta: Steering angle
    L: Wheelbase
    dt: Time step

    Returns:
    Updated position (x, y) and orientation (theta) of the robot.
    """
    x_new = x + v * math.cos(theta) * dt
    y_new = y + v * math.sin(theta) * dt
    theta_new = theta + (v / L) * math.tan(delta) * dt

    return x_new, y_new, theta_new


def convert_yup_to_zup(location_yup, quaternion_yup):
    # Define the 90-degree rotation about the x-axis as a quaternion
    q_rot = R.from_euler('x', 90, degrees=True).as_quat()

    # Convert the location vector from y-up to z-up
    # Rotation matrix for 90 degrees around x-axis
    R_90_x = np.array([[1, 0, 0],
                       [0, 0, 1],
                       [0, -1, 0]])

    location_zup = R_90_x.dot(location_yup)

    # Convert the quaternion from y-up to z-up
    # Quaternion multiplication: q_new = q_rot * q_orig * q_rot_inv
    q_yup = R.from_quat(quaternion_yup)
    q_rot_inv = R.from_quat(q_rot).inv()
    q_new = q_rot_inv * q_yup * R.from_quat(q_rot)
    quaternion_zup = q_new.as_quat()

    # Convert the new quaternion to Euler angles
    euler_angles_zup = q_new.as_euler('xyz', degrees=True)
    for i in range(3):
        euler_angles_zup[i] = np.floor(euler_angles_zup[i])
    return location_zup, euler_angles_zup


# Example usage
#location_yup = np.array([1, 2, 3])
#quaternion_yup = np.array([0.7071, 0, 0.7071, 0])  # Example quaternion

#location_zup, euler_angles_zup = convert_yup_to_zup(location_yup, quaternion_yup)

#print("Location in z-up:", location_zup)
#print("Euler angles in z-up:", euler_angles_zup)
#####

def only2(var):
    newvar = []
    for i in range(len(var)):  # Iterate through indices of var
        newvar.append(round(var[i], 2))  # Format var[i] to two decimal places
    return newvar


trajectory_x = []
trajectory_y = []
target_trajectory_x = []
target_trajectory_y = []


def receive_new_frame(data_frame: DataFrame):
    global i

    global num_frames
    num_frames += 1
    if num_frames % 10 == 0:
        x_c = None
        y_c = None
        rad_c = None
        cord_t = None
        # print(data_frame)
        # print (data_frame.rigid_bodies)
        for ms in data_frame.rigid_bodies:

            if (ms.id_num == 600):
                location_yup = np.array(ms.pos)
                quaternion_yup = np.array(ms.rot)
                location_zup, euler_angles_zup = convert_yup_to_zup(location_yup, quaternion_yup)
                only2(ms.pos)
                rad_c = math.radians(euler_angles_zup[2])
                x_c = only2(ms.pos)[0]
                y_c = only2(ms.pos)[2]
                print("-----CHASER-----")
                print(f"position chaser: {only2(ms.pos)}")
                print(f"rotation chaser:    {euler_angles_zup}")
                print(f"rad chaser:    {rad_c}")
                # print(f"rotation quaternion chaser:    {ms.rot}")
                print("\n")

            if (ms.id_num == 601):
                location_yup = np.array(ms.pos)

                quaternion_yup = np.array(ms.rot)
                location_zup, euler_angles_zup = convert_yup_to_zup(location_yup, quaternion_yup)
                rad_t = math.radians(euler_angles_zup[2])
                cord_t = (only2(ms.pos)[0], only2(ms.pos)[2], rad_t)

                print("-----TARGET-----")
                print(f"position target:   {only2(ms.pos)}")
                print(f"rotation target:    {euler_angles_zup}")
                print(f"rad target:    {rad_t}")
                print(f"cord target :    {cord_t}")
                # print(f"rotation quaternion target:    {ms.rot}")
                print("\n")
                print("-----------------------------------------------------------------\n")
        # Calculate the steering angle using Pure Pursuit
        print(type(cord_t[1]))

        delta, lookahead_point = pure_pursuit(x_c, y_c, rad_c, cord_t, 2)

        # Update the robot's state using the kinematic bicycle model
        x_r, y_r, theta = kinematic_bicycle_model(x_c, y_c, rad_c, 1, delta, 5, 0.2)
        # Store the trajectory
        trajectory_x.append(x_r)
        trajectory_y.append(y_r)
        target_trajectory_x.append(cord_t[0])
        target_trajectory_y.append(cord_t[1])

        # Print the details for each step
        print(f"Step {i}:")
        i += 1
        # print(
        #     f"Target Position: ({target_x:.2f}, {target_y:.2f}), Orientation: {math.degrees(target_theta):.2f} degrees, Velocity: {target_v:.2f}")
        # print(
        #     f"Robot Position: ({x_r:.2f}, {y_r:.2f}), Orientation: {math.degrees(theta):.2f} degrees, Velocity: {v:.2f}")
        # print(f"Steering Angle: {math.degrees(delta):.2f} degrees")
        # print(f"Lookahead Point: ({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f})")
        # print('-' * 50)
        # Example usage:

        #ip_address = "192.168.0.102" #(already defined)
        #port = 80 #(already defined)
        left_direction = "F"
        left_pwm = 250
        right_direction = "F"
        right_pwm = 250
        angle = 90

        response = send_motion_command(ip_address, port, left_direction, left_pwm, right_direction, right_pwm, angle)


def receive_new_desc(desc: DataDescriptions):
    print("Received data descriptions.")
    # print(desc)
    for ms in desc.marker_sets:
        if (ms.name == 'IOT_car'):
            print(desc)


num_frames = 0

if __name__ == "__main__":
    streaming_client = NatNetClient(server_ip_address="132.68.35.30",
                                    local_ip_address=socket.gethostbyname(socket.gethostname()), use_multicast=False)
    streaming_client.on_data_description_received_event.handlers.append(receive_new_desc)
    streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)

    with streaming_client:
        streaming_client.request_modeldef()

        for i in range(1):
            time.sleep(1)
            streaming_client.update_sync()
            print(f"Received {num_frames} frames in {i + 1}s")

send_motion_command(ip_address, port, "F", 0, "F", 0, 90)

# Plot the path and the robot's trajectory
plt.plot(target_trajectory_x, target_trajectory_y, 'r--', label='Target Path')
plt.plot(trajectory_x, trajectory_y, 'b-', label='Robot Trajectory')
plt.scatter(target_trajectory_x, target_trajectory_y, color='red')
plt.xlabel('X')
plt.ylabel('Y')
plt.xticks(range(6))
plt.yticks(range(6))
plt.legend()
plt.show()
