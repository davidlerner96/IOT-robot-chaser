import time
import socket
from natnet_client import DataDescriptions, DataFrame, NatNetClient
import algorithms
import chaser_data_handling
import commands
import conversion
import numpy as np
import math
import py_PurePursuit_intersect as ppi
from datetime import datetime
from conversion import dist
import matplotlib.pyplot as plt
from edgeCases import is_out_of_board
from edgeCases import get_speed


try:
    ip_address = socket.gethostbyname("esp32.local")
    print(f"The IP address of esp32.local is {ip_address}\n\n")
except socket.gaierror as e:
    print(f"Dear friend !!\nDNS resolution failed:\nPlease check the following:\n"
          f"1.The robot chaser is powered on. \n"
          f"2.The router is working properly. \n"
          f"3.The battery is full.  \n"
          f"4.The wifi configuration."
          f"\n\n\n{e}")
    exit()
except Exception as e:
    print(f"An unexpected error occurred: {e}")

port = 80

backwards_driving = False
close_break = False
stop = False

# y up -> x,z planar motion. yaw is around the third parameter.
i, num_frames = 0, 0
c_pos, c_rot, c_rad = 0, 0, 0
t_pos, t_rot, t_rad = 0, 0, 0
# Initialize lists to keep track of chaser and target positions
chaser_positions = []
target_positions = []
WB = 0.14
MAX_STEER = np.deg2rad(30.0)  # maximum steering angle [rad]
# initial state
state = 0
# initial yaw compensation
pp = 0
target_pos = 0
steering_angle = 0
prev_target_x = None
prev_target_y = None
prev_time = None
current_time = None
timestamp = None


def receive_new_desc(desc: DataDescriptions):
    print("Received data descriptions.")
    # print(desc)
    for ms in desc.marker_sets:
        if ms.name == 'IOT_car':
            print(desc)


def receive_new_frame(data_frame: DataFrame):
    """
    This function processes a new data frame to extract relevant information about the
    chaser and target positions, calculate the target velocity, and control the steering angle.
    """
    # Declare global variables that are used within this function
    global i, num_frames
    global c_pos, c_rot, c_rad
    global t_pos, t_rot, t_rad
    global state, pp
    global target_pos, steering_angle, stop
    global target_v, prev_target_x, prev_target_y
    global chaser_positions, target_positions  # Track positions
    global prev_time
    global WB
    # Increment frame counter to keep track of the number of frames processed
    num_frames += 1

    # Process every 100th frame to avoid excessive processing
    if num_frames % 20 == 0:
        # Iterate through the rigid bodies in the data frame to extract positions
        for ms in data_frame.rigid_bodies:
            if ms.id_num == 600:
                # Handle the chaser's data
                c_pos, c_rot, c_rad = chaser_data_handling.handle_frame(ms, "CHASER")
            if ms.id_num == 601:
                # Handle the target's data
                t_pos, t_rot, t_rad = chaser_data_handling.handle_frame(ms, "TARGET")
        if dist(c_pos[0] ,t_pos[0], c_pos[2] ,t_pos[2]) < 0.2:
            print(f"YEYYYYY \nThe robot chaser got the target.")
            commands.send_motion_command(ip_address, port, "F", 000, "F", 000, 95)
            plot_positions(chaser_positions, target_positions)
            exit()
        # Check if the chaser is within the board limits
        if is_out_of_board(c_pos[0], c_pos[2]):
            print(f"Chaser board limit fail. Check if the chaser robot is on the board.")
            commands.send_motion_command(ip_address, port, "F", 000, "F", 000, 95)
            exit()
        # Check if the target is within the board limits
        if is_out_of_board(t_pos[0], t_pos[2]):
            print(f"Target board limit fail. Check if the target robot is on the board.")
            commands.send_motion_command(ip_address, port, "F", 000, "F", 000, 95)
            exit()

        # Get the current time for velocity calculation
        current_time = datetime.now().timestamp()

        # If this is the first frame, initialize the target velocity
        if prev_time is None:
            target_v = 0
        else:
            # Calculate the time difference between the current and previous frames
            time_diff = current_time - prev_time
            # Calculate the target velocity based on the distance traveled over time
            target_v = dist(t_pos[0], prev_target_x, t_pos[2], prev_target_y) / time_diff

        # Update the previous time and target position with the current data
        prev_time = current_time
        prev_target_x = t_pos[0]
        prev_target_y = t_pos[2]

        # Store positions for plotting
        chaser_positions.append((c_pos[0], c_pos[2]))
        target_positions.append((t_pos[0], t_pos[2]))

        # Initialize the state of the chaser for forward and backward movement
        state = ppi.State(x=c_pos[0], y=c_pos[2], yaw=c_rad, v=0.70)
        state_b = ppi.State(x=c_pos[0], y=c_pos[2], yaw=c_rad + 3.14, v=-0.70)  # Reverse direction

        # Initialize the PurePursuit controller
        pp = ppi.PurePersuit_Controller(WB, MAX_STEER)
        target_pos = [t_pos[0], t_pos[2]]  # Update the target position

        # Flag to indicate if driving backward is necessary
        drive_b = False

        # Calculate steering angles for forward and backward movement
        steering_angle_f, alpha_f = pp.pure_pursuit_steer_control(state, target_pos, target_v)
        steering_angle_b, alpha_b = pp.pure_pursuit_steer_control(state_b, target_pos, target_v)

        # Determine the optimal steering direction (forward or backward)
        if backwards_driving:
            if abs(alpha_f) < abs(alpha_b):
                steering_degree = int(np.degrees(steering_angle_f)) + 95
            else:
                drive_b = True  # Set flag for driving backward
                steering_degree = -int(np.degrees(steering_angle_b)) + 95
        else:
            # Default to forward driving
            steering_degree = int(np.degrees(steering_angle_f)) + 95

        try:
            # Calculate the distance between the chaser and the target
            d = np.sqrt((c_pos[2] - t_pos[2]) ** 2 + (c_pos[0] - t_pos[0]) ** 2)
            # Check if the chaser is too close to the target and stop if necessary
            if close_break and (drive_b and d < 1 or not drive_b and d < 0.8):
                stop = True  # Set the stop flag
                res = commands.send_motion_command(ip_address, port, "F", 250, "F", 0, 90)
            else:
                # Send motion commands for driving forward or backward based on conditions
                if not drive_b:
                    right_speed, left_speed = get_speed(steering_degree-95)
                    res = commands.send_motion_command(ip_address, port, "F", left_speed, "F", right_speed, int(steering_degree))
                else:
                    res = commands.send_motion_command(ip_address, port, "B", 250, "B", 250, int(steering_degree))

            print(res)  # Print the result of the motion command
        except Exception as e:
            print("An error occurred:", e)  # Handle exceptions during motion commands

        time.sleep(0.1)  # Pause briefly between frames
        print("-----------------------------------------------------------------")

        # Call function to plot the positions
        #plot_positions(chaser_positions, target_positions)


def plot_positions(chaser_positions, target_positions):
    """
    Plots the chaser and target positions over time.

    Args:
        chaser_positions (list of tuples): The tracked chaser positions.
        target_positions (list of tuples): The tracked target positions.
    """
    plt.figure(figsize=(10, 6))
    chaser_x, chaser_y = zip(*chaser_positions)  # Unpack chaser positions
    target_x, target_y = zip(*target_positions)  # Unpack target positions

    plt.plot(chaser_x, chaser_y, label="Chaser Path", marker='o')  # Plot chaser path
    plt.plot(target_x, target_y, label="Target Path", marker='x')  # Plot target path

    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Chaser and Target Paths Over Time')
    plt.legend()  # Add legend to differentiate paths
    plt.grid(True)  # Add grid for better visualization
    plt.show()  # Display the plot


# connecting the client
streaming_client = NatNetClient(server_ip_address="132.68.35.30",
                                local_ip_address=socket.gethostbyname(socket.gethostname()),
                                use_multicast=False)
streaming_client.on_data_description_received_event.handlers.append(receive_new_desc)
streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)
# request the model definitions from the server, which causes it to send a data description packet
try:
    with streaming_client:
        streaming_client.request_modeldef()

        for i in range(100):
            if close_break and stop:
                break
            time.sleep(1)
            # Processing data synchronously
            streaming_client.update_sync()
            print(f"Received {num_frames} frames in {i + 1}s")
except ConnectionResetError as e:
    print(
        f"Dear friend !!\nOptitrack connection failed:\nPlease check if the Optitrack system is on and streaming.\n\n\n{e}")
    exit()
    # Handle the error or retry logic here
except Exception as e:
    print(f"An unexpected error occurred: {e}")
    # Handle other exceptions here
commands.send_motion_command(ip_address, port, "F", 0, "F", 0, 90)