import numpy as np
from scipy.spatial.transform import Rotation as R


def normalize_angle(angle):
    """Normalize the angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


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
# location_yup = np.array([1, 2, 3])
# quaternion_yup = np.array([0.7071, 0, 0.7071, 0])  # Example quaternion

# location_zup, euler_angles_zup = convert_yup_to_zup(location_yup, quaternion_yup)

# print("Location in z-up:", location_zup)
# print("Euler angles in z-up:", euler_angles_zup)
#####


def only2(var):
    new_var = []
    for i in range(len(var)):  # Iterate through indices of var
        new_var.append(round(var[i], 2))  # Format var[i] to two decimal places
    return new_var
