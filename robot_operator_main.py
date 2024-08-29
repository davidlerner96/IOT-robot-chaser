import time
import socket
from natnet_client import DataDescriptions, DataFrame, NatNetClient
import algorithms
import chaser_data_handling
import commands
import conversion
from matplotlib import pyplot as plt

ip_address = socket.gethostbyname("esp32.local")
port = 80

i, num_frames = 0, 0
c_pos, c_rot, c_rad = 0, 0, 0
t_pos, t_rot, t_rad = 0, 0, 0
trajectory_x = []
trajectory_y = []
target_trajectory_x = []
target_trajectory_y = []



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
    num_frames += 1
    if num_frames % 10 == 0:
        # print(data_frame)
        # print (data_frame.rigid_bodies)
        for ms in data_frame.rigid_bodies:
            if ms.id_num == 600:
                c_pos, c_rot, c_rad = chaser_data_handling.handle_frame(ms, "CHASER")
            if ms.id_num == 601:
                t_pos, t_rot, t_rad = chaser_data_handling.handle_frame(ms, "TARGET")
        print("Fill the array for animation")

        print("Algorithm Goes here")

        print("-----------------------------------------------------------------")


# connecting the client
streaming_client = NatNetClient(server_ip_address="132.68.35.30",
                                local_ip_address=socket.gethostbyname(socket.gethostname()),
                                use_multicast=False)
streaming_client.on_data_description_received_event.handlers.append(receive_new_desc)
streaming_client.on_data_frame_received_event.handlers.append(receive_new_frame)
# request the model definitions from the server, which causes it to send a data description packet
with streaming_client:
    streaming_client.request_modeldef()

    for i in range(3):
        time.sleep(1)
        # Processing data synchronously
        streaming_client.update_sync()
        print(f"Received {num_frames} frames in {i + 1}s")

commands.send_motion_command(ip_address, port, "F", 0, "F", 0, 90)
