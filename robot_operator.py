import time
import socket
from natnet_client import DataDescriptions, DataFrame, NatNetClient
import algorithms
import chaser_data_handling
import commands
import conversion
from matplotlib import pyplot as plt
import numpy as np
import math
import py_PurePursuit_intersect as ppi






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


port = 80

backwards_driving = False
close_break = False
stop = False

# y up -> x,z planar motion. yaw is around the third parameter.
i, num_frames = 0, 0
c_pos, c_rot, c_rad = 0, 0, 0
t_pos, t_rot, t_rad = 0, 0, 0
trajectory_x = []
trajectory_z = []
target_trajectory_x = []
target_trajectory_z = []

WB = 0.14
MAX_STEER = np.deg2rad(30.0)  # maximum steering angle [rad]
# initial state
state = 0
# initial yaw compensation
pp = 0
target_pos = 0
steering_angle = 0


def receive_new_desc(desc: DataDescriptions):
    print("Received data descriptions.")
    # print(desc)
    for ms in desc.marker_sets:
        if ms.name == 'IOT_car':
            print(desc)


def receive_new_frame(data_frame: DataFrame):
    global i
    global num_frames
    global c_pos, c_rot, c_rad
    global t_pos, t_rot, t_rad
    global state
    global pp
    global target_pos
    global steering_angle
    global num_frames
    global stop
    num_frames += 1
    if num_frames % 100 == 0:
        # print(data_frame)
        # print (data_frame.rigid_bodies)
        for ms in data_frame.rigid_bodies:
            if ms.id_num == 600:
                c_pos, c_rot, c_rad = chaser_data_handling.handle_frame(ms, "CHASER")
            if ms.id_num == 601:
                t_pos, t_rot, t_rad = chaser_data_handling.handle_frame(ms, "TARGET")
        print("Fill the array for animation")

        # t_pos[2] = c_pos[2] - (c_pos[2] - t_pos[2]) * 9 / 10
        # t_pos[0] = c_pos[0] - (c_pos[0] - t_pos[0]) * 9 / 10
        print(t_pos)

        print("Algorithm Goes here")
        state = ppi.State(x=c_pos[0], y=c_pos[2], yaw=c_rad, v=0.70)
        state_b = ppi.State(x=c_pos[0], y=c_pos[2], yaw=c_rad + 3.14, v=-0.70)
        # initial yaw compensation
        pp = ppi.PurePersuit_Controller(WB, MAX_STEER)
        target_pos = [t_pos[0], t_pos[2]]

        drive_b = False
        steering_angle_f, alpha_f = pp.pure_pursuit_steer_control(state, target_pos)
        steering_angle_b, alpha_b = pp.pure_pursuit_steer_control(state_b, target_pos)
        if backwards_driving:
            if abs(alpha_f) < abs(alpha_b):
                steering_degree = int(np.degrees(steering_angle_f)) + 95
            else:
                drive_b = True
                steering_degree = -int(np.degrees(steering_angle_b)) + 95

        else:
            steering_degree = int(np.degrees(steering_angle_f)) + 95

        print(steering_degree)

        res = None
        try:
            # t_pos[2] = c_pos[2] - (c_pos[2] - t_pos[2]) * 9 / 10
            # t_pos[0] = c_pos[0] - (c_pos[0] - t_pos[0]) * 9 / 10
            d = np.sqrt(pow((c_pos[2] - t_pos[2]), 2) + pow((c_pos[0] - t_pos[0]), 2))

            if close_break and (drive_b and d < 1 or not drive_b and d < 0.8):
                print("Too close! Stopping.")
                stop = True
                res = commands.send_motion_command(ip_address, port, "F", 250, "F", 0, 90)
            else:
                if not drive_b:
                    res = commands.send_motion_command(ip_address, port, "F", 250, "F", 250, int(steering_degree))
                else:
                    res = commands.send_motion_command(ip_address, port, "B", 250, "B", 250, int(steering_degree))

            print(res)
        except Exception as e:
            print("Whyyyyyyyyyyyyyyy")

        time.sleep(0.1)
        print("Plotting goes here")

        print("-----------------------------------------------------------------")


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
    print(f"Dear friend !!\nOptitrack connection failed:\nPlease check if the Optitrack system is on and streaming.\n\n\n{e}")
    exit()
    # Handle the error or retry logic here
except Exception as e:
    print(f"An unexpected error occurred: {e}")
    # Handle other exceptions here
commands.send_motion_command(ip_address, port, "F", 0, "F", 0, 90)
