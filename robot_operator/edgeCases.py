limit_X = 4.45
limit_Y = 2


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
        right = 250 - int(80/30*abs(steering_degree))
        left = 250
    else:
        right = 250
        left = 250 - int(80 / 30 * abs(steering_degree))

    print(right, left)
    return right, left
