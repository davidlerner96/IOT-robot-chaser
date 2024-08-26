import math
import matplotlib.pyplot as plt

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

# Simulation parameters
x_r, y_r = 2, 3  # Robot's initial rear axle midpoint
theta = math.pi / 4  # Robot's initial orientation in radians
v = 1.0  # Robot's velocity
L = 2  # Wheelbase
dt = 0.1  # Time step
lookahead_distance = 2  # Initial lookahead distance

# Target's initial state
target_x, target_y = 0, 0
target_theta = 0  # Target's initial orientation
target_v = 1.5  # Initial target velocity

# Lists to store the trajectory for plotting
trajectory_x = [x_r]
trajectory_y = [y_r]
target_trajectory_x = [target_x]
target_trajectory_y = [target_y]

# Run the simulation for a certain number of time steps
for step in range(600):
    # Update target's position
    prev_target_x, prev_target_y = target_x, target_y
    target_x += target_v * math.cos(target_theta) * dt
    target_y += target_v * math.sin(target_theta) * dt

    # Add some curve to the target's path
    target_theta += 0.1 * math.sin(0.1 * step)

    # Vary the target's velocity to simulate acceleration and deceleration
    if step % 20 == 0:
        target_v = max(0.5, target_v + (0.5 * (-1) ** (step // 20)))

    target = (target_x, target_y, target_theta)

    # Calculate the steering angle using Pure Pursuit
    delta, lookahead_point = pure_pursuit(x_r, y_r, theta, target, lookahead_distance)

    # Update the robot's state using the kinematic bicycle model
    x_r, y_r, theta = kinematic_bicycle_model(x_r, y_r, theta, v, delta, L, dt)

    # Store the trajectory
    trajectory_x.append(x_r)
    trajectory_y.append(y_r)
    target_trajectory_x.append(target_x)
    target_trajectory_y.append(target_y)

    # Print the details for each step
    print(f"Step {step}:")
    print(f"Target Position: ({target_x:.2f}, {target_y:.2f}), Orientation: {math.degrees(target_theta):.2f} degrees, Velocity: {target_v:.2f}")
    print(f"Robot Position: ({x_r:.2f}, {y_r:.2f}), Orientation: {math.degrees(theta):.2f} degrees, Velocity: {v:.2f}")
    print(f"Steering Angle: {math.degrees(delta):.2f} degrees")
    print(f"Lookahead Point: ({lookahead_point[0]:.2f}, {lookahead_point[1]:.2f})")
    print('-' * 50)

# Plot the path and the robot's trajectory
plt.plot(target_trajectory_x, target_trajectory_y, 'r--', label='Target Path')
plt.plot(trajectory_x, trajectory_y, 'b-', label='Robot Trajectory')
plt.scatter(target_trajectory_x, target_trajectory_y, color='red')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.show()
