import numpy as np
import math
import matplotlib.pyplot as plt


class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.WB = 0.14
        self.yaw = yaw
        self.x = x + ((self.WB / 3) * math.cos(self.yaw))
        self.y = y + ((self.WB / 2) * math.sin(self.yaw))

        self.v = v
        self.rear_x = self.x - ((self.WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.WB / 2) * math.sin(self.yaw))
        self.predelta = 0.0


class PurePersuit_Controller(object):
    def __init__(self, WB=0.14, MAX_STEER=np.deg2rad(30.0), ):
        self.WB = WB
        self.MAX_STEER = MAX_STEER
        self.Lookahead = 0.1

    def pure_pursuit_steer_control(self, state, target_pos, target_v):
        # drive_b = False
        target_x, target_y = target_pos
        Lf = ((target_y - state.rear_y) ** 2 + (target_x - state.rear_x) ** 2) ** 0.5 + state.WB + target_v*self.Lookahead
        alpha = math.atan2(target_y - state.rear_y, target_x - state.rear_x) - state.yaw
        delta = math.atan2(2.0 * self.WB * math.sin(alpha) / Lf, 1.0)*1.2
        # alpha_b = abs(alpha) + 3.14
        # if alpha_b < abs(alpha):
        #     drive_b = True
        print(f"delta :  {np.degrees(delta/1.2)}")
        if delta >= self.MAX_STEER:
            delta = self.MAX_STEER
        elif delta <= -self.MAX_STEER:
            delta = -self.MAX_STEER
        return delta, alpha


class Odom(object):
    def __init__(self, wb):
        self.wheelbase = wb

    def propagate(self, velocity, steering):
        theta_dot = velocity * np.tan(steering) / self.wheelbase
        dt = 0.01
        waypoints_x = []
        waypoints_y = []
        waypoints_theta = []
        waypoints_x.append(self.x)
        waypoints_y.append(self.y)
        waypoints_theta.append(self.theta)
        # for _ in range(int(time/dt)):
        while ((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2) ** 0.5 > 0.3:
            self.theta += theta_dot * dt
            x_dot = velocity * np.cos(self.theta)
            y_dot = velocity * np.sin(self.theta)
            self.x += x_dot * dt
            self.y += y_dot * dt
            waypoints_x.append(self.x)
            waypoints_y.append(self.y)
            waypoints_theta.append(self.theta)
        self.x = waypoints_x[-1]
        self.y = waypoints_y[-1]
        self.theta = waypoints_theta[-1]
        return [waypoints_x, waypoints_y, waypoints_theta]

    def plot_trajectory(self, state, steering, target_pos):
        self.x, self.y, self.theta = state.x, state.y, state.yaw
        self.target_x, self.target_y = target_pos
        fig = plt.figure()
        ax = fig.add_subplot()
        velocity = 10.0

        for _ in range(30):
            waypoints = self.propagate(velocity=velocity, steering=steering)
            print(waypoints[0])
            ax.scatter(waypoints[0], waypoints[1])
        ax.set_aspect('equal', 'box')
        plt.show()


def main():
    WB = 0.14
    MAX_STEER = np.deg2rad(30.0)  # maximum steering angle [rad]
    # initial state
    state = State(x=0.0, y=0.0, yaw=0.0, v=1)
    # initial yaw compensation
    pp = PurePersuit_Controller(WB, MAX_STEER)
    target_pos = [3, 5]
    steering_angle = pp.pure_pursuit_steer_control(state, target_pos)

    # simulation
    odom = Odom(WB)
    odom.plot_trajectory(state, steering_angle, target_pos)


if __name__ == '__main__':
    main()