import algorithms
import conversion
import commands
import numpy as np
import math

# i, num_frames = 0, 0
#
# trajectory_x = []
# trajectory_y = []
# target_trajectory_x = []
# target_trajectory_y = []


def handle_frame(ms, name):
    location_yup = np.array(ms.pos)
    quaternion_yup = np.array(ms.rot)
    location_zup, euler_angles_zup = conversion.convert_yup_to_zup(location_yup, quaternion_yup)
    conversion.only2(ms.pos)
    rad = round(math.radians(euler_angles_zup[2]), 2)
    print(f"-----{name}-----")
    print(f"position: {conversion.only2(ms.pos)}")
    print(f"rotation:    {euler_angles_zup}")
    print(f"rad:    {rad}")
    # print(f"rotation quaternion chaser:    {ms.rot}")
    return conversion.only2(ms.pos), euler_angles_zup, rad


