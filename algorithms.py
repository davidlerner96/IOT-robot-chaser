import math


def normalize_angle(angle):
    """Normalize the angle to the range [-pi, pi]."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


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
