import requests


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


# Example usage:
ip_address = "192.168.1.100"
port = 8080
left_direction = "F"
left_pwm = 150
right_direction = "B"
right_pwm = 200
angle = 75

response = send_motion_command(ip_address, port, left_direction, left_pwm, right_direction, right_pwm, angle)
print(response.status_code)
print(response.text)
