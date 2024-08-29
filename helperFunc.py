limit_X = 4.45
limit_Y = 2
import math

#if True : The robot out of the borders of the board
#if False : The robot in the borders of the board
def is_out_of_board(cord_x, cord_y):
    if limit_X < cord_x or limit_X < -cord_x:
        return True
    if limit_Y < cord_y or limit_Y < -cord_y:
        return True
    return False


def get_speed(steering_degree):
    if steering_degree > 0:
        right = 250 - int(70 / 30 * abs(steering_degree))
        left = 250
    else:
        right = 250
        left = 250 - int(70 / 30 * abs(steering_degree))

    print(right, left)
    return right, left

def dist(x1, x2, y1, y2):
    """
    Calculate the Euclidean distance between two points (x1, y1) and (x2, y2).

    Args:
        x1 (float): x-coordinate of the first point.
        y1 (float): y-coordinate of the first point.
        x2 (float): x-coordinate of the second point.
        y2 (float): y-coordinate of the second point.

    Returns:
        float: The Euclidean distance between the two points.
    """
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)