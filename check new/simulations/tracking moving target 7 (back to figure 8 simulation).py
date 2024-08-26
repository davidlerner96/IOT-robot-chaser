import numpy as np
import matplotlib.pyplot as plt

# Constants
dt = 0.1  # Time step
initial_target_speed = 2.0  # Initial target speed
max_target_speed = 4.0  # Maximum target speed
min_target_speed = 0.5  # Minimum target speed
robot_speed = 1.0  # Robot speed
wheelbase_length = 2.5  # Distance between the front and rear axles
max_steering_angle = np.radians(180)  # Maximum steering angle in radians
acceleration_rate = 0.05  # Target acceleration rate
deceleration_rate = 0.03  # Target deceleration rate
lookahead_distance_initial = 5.0  # Initial lookahead distance

# Initial conditions
target_position = np.array([0.0, 0.0])
robot_position = np.array([2.0, 3.0])
robot_orientation = np.radians(45.0)
lookahead_distance = lookahead_distance_initial

# Trajectories
target_trajectory = []
robot_trajectory = [robot_position.copy()]


# Function to calculate the lookahead point
def get_lookahead_point(target_position, target_orientation, lookahead_distance):
    return target_position + lookahead_distance * np.array([np.cos(target_orientation), np.sin(target_orientation)])


# Function to calculate the curvature for the lookahead distance adjustment
def calculate_curvature(target_trajectory):
    if len(target_trajectory) < 3:
        return 0.0
    p1, p2, p3 = target_trajectory[-3:]
    v1, v2 = p2 - p1, p3 - p2
    curvature = np.abs(np.cross(v1, v2)) / (np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6)
    return curvature


# Function to calculate the steering angle
def pure_pursuit_control(robot_position, robot_orientation, lookahead_point, lookahead_distance):
    dx = lookahead_point[0] - robot_position[0]
    dy = lookahead_point[1] - robot_position[1]
    target_angle = np.arctan2(dy, dx)
    alpha = target_angle - robot_orientation
    Ld = lookahead_distance
    steering_angle = np.arctan2(2 * wheelbase_length * np.sin(alpha), Ld)
    return np.clip(steering_angle, -max_steering_angle, max_steering_angle)


# Function to generate the figure-eight path
def generate_figure_eight_path(t, a=5, b=2):
    x = a * np.sin(t)
    y = b * np.sin(t) * np.cos(t)
    return np.array([x, y])


# Simulation loop
target_speed = initial_target_speed
for step in range(200):
    t = step * dt

    # Update target speed (acceleration and deceleration)
    if step < 100:
        target_speed = min(target_speed + acceleration_rate * dt, max_target_speed)
    else:
        target_speed = max(target_speed - deceleration_rate * dt, min_target_speed)
    
    # Update target position along the figure-eight path
    target_position = generate_figure_eight_path(t)
    target_orientation = np.arctan2(target_position[1],
                                    target_position[0])  # Calculate orientation based on path tangent

    # Calculate lookahead distance based on velocity and curvature
    curvature = calculate_curvature(target_trajectory)
    lookahead_distance = lookahead_distance_initial + robot_speed * curvature

    # Calculate lookahead point
    lookahead_point = get_lookahead_point(target_position, target_orientation, lookahead_distance)

    # Calculate steering angle
    steering_angle = pure_pursuit_control(robot_position, robot_orientation, lookahead_point, lookahead_distance)

    # Update robot position and orientation
    robot_position += robot_speed * dt * np.array([np.cos(robot_orientation), np.sin(robot_orientation)])
    robot_orientation += (robot_speed / wheelbase_length) * np.tan(steering_angle) * dt

    # Store trajectories
    target_trajectory.append(target_position.copy())
    robot_trajectory.append(robot_position.copy())

    # Debugging output
    print(f"Step {step}:")
    print(
        f"Target Position: ({target_position[0]:.2f}, {target_position[1]:.2f}), Orientation: {np.degrees(target_orientation):.2f} degrees, Speed: {target_speed:.2f}")
    print(
        f"Robot Position: ({robot_position[0]:.2f}, {robot_position[1]:.2f}), Orientation: {np.degrees(robot_orientation):.2f} degrees, Speed: {robot_speed:.2f}")
    print(f"Steering Angle: {np.degrees(steering_angle):.2f} degrees")
    print(f"Lookahead Point: ({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f})")
    print(f"Lookahead Distance: {lookahead_distance:.2f}")
    print("-" * 50)

# Plotting
target_trajectory = np.array(target_trajectory)
robot_trajectory = np.array(robot_trajectory)

plt.figure(figsize=(10, 5))
plt.plot(target_trajectory[:, 0], target_trajectory[:, 1], 'r-', label='Target Trajectory')
plt.plot(robot_trajectory[:, 0], robot_trajectory[:, 1], 'b-', label='Robot Trajectory')
plt.plot(target_trajectory[0, 0], target_trajectory[0, 1], 'ro', label='Target Start')
plt.plot(robot_trajectory[0, 0], robot_trajectory[0, 1], 'bo', label='Robot Start')
plt.xlabel('X position')
plt.ylabel('Y position')
plt.legend()
plt.grid(True)
plt.title('Pure Pursuit Trajectories')
plt.show()
