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
from helperFunc import dist
import matplotlib.pyplot as plt
from helperFunc import is_out_of_board
from helperFunc import get_speed
from commands import send_led_error_command
ip_address = None
try:
    ip_address = socket.gethostbyname("esp32.local")
    send_led_error_command(ip_address, 80, 4, "low")
    send_led_error_command(ip_address, 80, 15, "low")
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

backwards_driving = True
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
drive_b = None


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
    global drive_b
    # Increment frame counter to keep track of the number of frames processed
    num_frames += 1

    # Process every 100th frame to avoid excessive processing
    if (num_frames % 15 == 0):
        # Iterate through the rigid bodies in the data frame to extract positions
        for ms in data_frame.rigid_bodies:
            if ms.id_num == 600:
                # Handle the chaser's data
                c_pos, c_rot, c_rad = chaser_data_handling.handle_frame(ms, "CHASER")
            if ms.id_num == 601:
                # Handle the target's data
                t_pos, t_rot, t_rad = chaser_data_handling.handle_frame(ms, "TARGET")
        if dist(c_pos[0], t_pos[0], c_pos[2], t_pos[2]) < 0.25:
            print(f"YEYYYYY \nThe robot chaser got the target.")
            commands.send_motion_command(ip_address, port, "F", 000, "F", 000, 95)
            plot_positions(chaser_positions, target_positions)
            exit()
        # Check if the chaser is within the board limits
        if is_out_of_board(c_pos[0], c_pos[2]):
            print(f"Chaser board limit fail. Check if the chaser robot is on the board.")
            commands.send_motion_command(ip_address, port, "F", 000, "F", 000, 95)
            send_led_error_command(ip_address, port, 4, "high")
            exit()
        # Check if the target is within the board limits
        if is_out_of_board(t_pos[0], t_pos[2]):
            print(f"Target board limit fail. Check if the target robot is on the board.")
            commands.send_motion_command(ip_address, port, "F", 000, "F", 000, 95)
            send_led_error_command(ip_address, port, 4, "high")
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

        # Calculate steering angles for forward and backward movement
        steering_angle_f, alpha_f = pp.pure_pursuit_steer_control(state, target_pos, target_v)
        steering_angle_b, alpha_b = pp.pure_pursuit_steer_control(state_b, target_pos, target_v)
        # Determine the optimal steering direction (forward or backward)

        if backwards_driving:
            limit_deg = 15 * 3.14 / 180


            # print(f"alpha_b = {alpha_b}")
            alpha_f = alpha_f % (np.pi * 2)
            # print(f"alpha_f = {alpha_f}")
            if not drive_b and (abs(alpha_f) - limit_deg < np.pi / 2 or abs(alpha_f) + limit_deg > 3 * np.pi / 2):
                steering_degree = int(np.degrees(steering_angle_f)) + 95
                drive_b = False
            elif not drive_b and (abs(alpha_f) - limit_deg >= np.pi / 2 and abs(alpha_f) + limit_deg <= 3 * np.pi / 2):
                steering_degree = -int(np.degrees(steering_angle_b)) + 95
                drive_b = True  # Set flag for driving backward
            elif drive_b and (abs(alpha_f) + limit_deg < np.pi / 2 or abs(alpha_f) - limit_deg > 3 * np.pi / 2):
                steering_degree = int(np.degrees(steering_angle_f)) + 95
                drive_b = False
            else:
                steering_degree = -int(np.degrees(steering_angle_b)) + 95
                drive_b = True  # Set flag for driving backward

        else:
            # Default to forward driving
            steering_degree = int(np.degrees(steering_angle_f)) + 95

        try:
            # Calculate the distance between the chaser and the target
            # d = np.sqrt((c_pos[2] - t_pos[2]) ** 2 + (c_pos[0] - t_pos[0]) ** 2)
            # # Check if the chaser is too close to the target and stop if necessary
            # if close_break and (drive_b and d < 1 or not drive_b and d < 0.8):
            #     stop = True  # Set the stop flag
            #     res = commands.send_motion_command(ip_address, port, "F", 250, "F", 0, 90)
            print(f"steering_degree : {steering_degree}")
            # Send motion commands for driving forward or backward based on conditions
            if not drive_b:
                right_speed, left_speed = get_speed(steering_degree - 95)
                res = commands.send_motion_command(ip_address, port, "F", left_speed, "F", right_speed,
                                                   int(steering_degree))
            else:
                res = commands.send_motion_command(ip_address, port, "B", 250, "B", 250, int(steering_degree))

            print(res)  # Print the result of the motion command
        except Exception as e:
            print("An error occurred:", e)  # Handle exceptions during motion commands

        time.sleep(0.1)  # Pause briefly between frames
        print("-----------------------------------------------------------------")

        # Call function to plot the positions
        #plot_positions(chaser_positions, target_positions)


import matplotlib.pyplot as plt


def plot_positions(chaser_positions, target_positions):
    """
    Plots the chaser and target positions over time.

    Args:
        chaser_positions (list of tuples): The tracked chaser positions, each tuple is (x, y).
        target_positions (list of tuples): The tracked target positions, each tuple is (x, y).
    """
    plt.figure(figsize=(6, 10))  # Set the figure size to 6x10 inches

    # Unpack chaser and target positions into x and y components
    chaser_x, chaser_y = zip(*chaser_positions)
    target_x, target_y = zip(*target_positions)

    # Plot chaser path with 'o' markers
    plt.plot(chaser_y, chaser_x, label="Chaser Path", marker='o')

    # Plot target path with 'x' markers
    plt.plot(target_y, target_x, label="Target Path", marker='x')

    # Label the x-axis and y-axis
    plt.xlabel('X Position')
    plt.ylabel('Y Position')

    # Set the plot title
    plt.title('Chaser and Target Paths Over Time')

    # Add legend to the plot for clarity
    plt.legend()

    # Add grid lines to the plot for better readability
    plt.grid(True)

    # Set the x-axis limits from -3 to 3
    plt.xlim(-3, 3)

    # Set the y-axis limits from -5 to 5
    plt.ylim(-5, 5)

    # Adjust the aspect ratio of the plot to fit the specified limits
    plt.gca().set_aspect('auto')

    # Display the plot
    plt.show()



# Initialize the NatNetClient with server and local IP addresses
streaming_client = NatNetClient(
    server_ip_address="132.68.35.30",  # IP address of the OptiTrack server
    local_ip_address=socket.gethostbyname(socket.gethostname()),  # Local IP address
    use_multicast=False  # Use unicast instead of multicast for communication
)

# Attach event handlers to handle data description and data frame reception
streaming_client.on_data_description_received_event.handlers.append(receive_new_desc)
streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)

# Request model definitions from the server to trigger a data description packet
try:
    with streaming_client:
        # Request the model definitions from the OptiTrack server
        streaming_client.request_modeldef()

        # Loop to receive and process frames for a specific duration
        for i in range(100):
            # Check if a condition to stop processing is met
            if close_break and stop:
                break

            # Sleep for 1 second before updating
            time.sleep(1)

            # Update the streaming client to process incoming data synchronously
            streaming_client.update_sync()

            # Print the number of frames received
            print(f"Received {num_frames} frames in {i + 1}s")

# Handle connection-related errors specifically
except ConnectionResetError as e:
    print(
        f"Dear friend !!\nOptitrack connection failed:\nPlease check if the Optitrack system is on and streaming.\n\n\n{e}")
    # Send an LED error command to indicate an issue
    send_led_error_command(ip_address, 80, 15, "high")
    exit()  # Exit the program

# Handle any other unexpected exceptions
except Exception as e:
    print(f"An unexpected error occurred: {e}")
    # Handle other exceptions, possibly with logging or retry logic here

# (Commented out) Send a motion command - Uncomment and customize as needed
# commands.send_motion_command(ip_address, port, "F", 0, "F", 0, 90)

